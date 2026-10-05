#!/usr/bin/env python3
"""circular_formation.launch.py -- N TurtleBot3 Burgers on a circle.

Launches (Gazebo Harmonic, ROS 2 Jazzy):
  1. Gazebo with worlds/formation_world.sdf
  2. /clock bridge (gz -> ROS), immediately
  3. spawn N namespaced burgers (robot_0 .. robot_{N-1}) at `spawn_delay`
  4. ros_gz_bridge: per-robot cmd_vel (ROS -> gz) and odom (gz -> ROS)
  5. one formation_controller per robot
  6. formation_monitor (metrics + RViz markers)
  7. RViz2 only with rviz:=true

Spawn/bridge/namespace conventions follow NSKsim's swarm_sim.launch.py.

    ros2 launch circular_formation circular_formation.launch.py \
        num_robots:=6 radius:=2.5 omega:=0.05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from circular_formation.robot_model import write_namespaced_burger_sdf
from circular_formation.spawn_layout import spawn_poses

WORLD_NAME = 'formation_world'


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg = get_package_share_directory('circular_formation')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    stock_sdf = os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf')

    n = int(arg('num_robots'))
    if n < 2:
        raise RuntimeError('num_robots must be >= 2 for a formation')
    radius = float(arg('radius'))
    omega = float(arg('omega'))
    center = (float(arg('center_x')), float(arg('center_y')))
    spawn_delay = float(arg('spawn_delay'))
    headless = arg('headless').lower() in ('true', '1')

    poses = spawn_poses(n, arg('spawn_layout'), float(arg('spawn_scale')),
                        center)
    spawn_xs = [p[0] for p in poses]
    spawn_ys = [p[1] for p in poses]
    spawn_yaws = [p[2] for p in poses]

    world_file = os.path.join(pkg, 'worlds', f'{WORLD_NAME}.sdf')
    params_file = arg('params_file')

    shared_params = {
        'use_sim_time': True,
        'num_robots': n,
        'spawn_xs': spawn_xs,
        'spawn_ys': spawn_ys,
        'spawn_yaws': spawn_yaws,
        'radius': radius,
        'center_mode': arg('center_mode'),
        'center_x': center[0],
        'center_y': center[1],
    }

    actions = [
        # The burger's meshes are model://turtlebot3_common/...; the gz server
        # must be able to resolve them.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(tb3, 'models')),
        ExecuteProcess(
            cmd=['gz', 'sim'] + (['-s'] if headless else []) + ['-r',
                                                                world_file],
            output='screen'),
        # /clock: its own bridge, use_sim_time False (it publishes the clock).
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': False}], output='screen'),
    ]

    # Spawns
    actions.append(TimerAction(period=spawn_delay, actions=[
        Node(package='ros_gz_sim', executable='create',
             name=f'spawn_robot_{i}',
             arguments=['-world', WORLD_NAME,
                        '-file', write_namespaced_burger_sdf(stock_sdf, i),
                        '-name', f'robot_{i}',
                        '-x', str(x), '-y', str(y), '-z', '0.01',
                        '-Y', str(yaw)],
             output='screen')
        for i, (x, y, yaw) in enumerate(poses)
    ]))

    # Bridge: '[' = gz->ROS, ']' = ROS->gz
    bridge_specs = []
    for i in range(n):
        bridge_specs += [
            f'/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/robot_{i}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ]
    actions.append(TimerAction(period=spawn_delay + 1.0, actions=[
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='formation_bridge', arguments=bridge_specs,
             parameters=[{'use_sim_time': True}], output='screen'),
    ]))

    # Controllers + monitor
    controllers = [
        Node(package='circular_formation', executable='formation_controller',
             name='formation_controller', namespace=f'robot_{i}',
             parameters=[params_file, dict(shared_params, robot_id=i,
                                           omega=omega)],
             output='screen')
        for i in range(n)
    ]
    controllers.append(
        Node(package='circular_formation', executable='formation_monitor',
             name='formation_monitor',
             parameters=[params_file, shared_params], output='screen'))
    actions.append(TimerAction(period=spawn_delay + 3.0, actions=controllers))

    if arg('rviz').lower() in ('true', '1'):
        actions += [
            # RViz needs the fixed frame to exist; the markers live in 'world'.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='world_anchor',
                 arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
                 parameters=[{'use_sim_time': True}]),
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', os.path.join(pkg, 'rviz',
                                               'circular_formation.rviz')],
                 parameters=[{'use_sim_time': True}], output='screen'),
        ]
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('circular_formation')

    def declare(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default,
                                     description=desc)

    return LaunchDescription([
        declare('num_robots', '5', 'number of robots (>= 2)'),
        declare('radius', '2.0', 'formation circle radius [m]'),
        declare('omega', '0.05',
                'rotation rate of the formation [rad/s], CCW +; 0 = static'),
        declare('center_mode', 'centroid',
                "'centroid' (leaderless estimate) or 'fixed'"),
        declare('center_x', '0.0', 'fixed centre x [m] (also spawn centre)'),
        declare('center_y', '0.0', 'fixed centre y [m] (also spawn centre)'),
        declare('spawn_layout', 'spiral', "'spiral' scatter or 'ring'"),
        declare('spawn_scale', '0.8', 'spawn pattern scale [m]'),
        declare('spawn_delay', '3.0', 'seconds before spawning robots'),
        declare('headless', 'false', 'run gz server without the GUI'),
        declare('rviz', 'false',
                'start RViz2 (leave off for timed runs; it slows the sim)'),
        declare('params_file',
                os.path.join(pkg, 'config', 'circular_formation.yaml'),
                'controller / monitor parameter file'),
        OpaqueFunction(function=_launch_setup),
    ])
