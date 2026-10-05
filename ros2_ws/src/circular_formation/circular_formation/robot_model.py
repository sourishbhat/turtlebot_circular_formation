"""Namespaced TurtleBot3 Burger SDF for multi-robot Gazebo Harmonic runs.

Adapted from NSKsim's make_namespaced_burger_sdf (swarm_sim.launch.py), cut
down to what circular formation needs: velocity command in, odometry out.

The stock model publishes everything on bare topics (cmd_vel, odom, ...), so
N copies would all listen to the same command. Each replacement below is an
exact string that must occur exactly once in the stock file; a stock-model
update that reformats one fails loudly instead of silently leaving robots
sharing a topic.

The lidar and IMU are stripped by default: formation control uses only
odometry, and removing them means the world needs no Sensors/ogre2 server
system. Pass strip_sensors=False (and add the Sensors + Imu systems to the
world) if you want them back; their topics are then namespaced too.
"""

import re
import tempfile

_SENSOR_RE = re.compile(r'[ \t]*<sensor\b[^>]*>.*?</sensor>[ \t]*\r?\n?',
                        re.DOTALL)


def _replace_once(text: str, old: str, new: str) -> str:
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f'expected exactly one {old!r} in the stock burger SDF, found '
            f'{n}; the TurtleBot3 model changed -- update robot_model.py.')
    return text.replace(old, new)


def namespace_burger_sdf(stock_sdf: str, robot_id: int,
                         strip_sensors: bool = True) -> str:
    """Return the burger SDF text with every topic/frame under robot_<id>."""
    ns = f'robot_{robot_id}'
    text = stock_sdf
    pairs = [
        ('<topic>cmd_vel</topic>', f'<topic>/{ns}/cmd_vel</topic>'),
        ('<odom_topic>odom</odom_topic>', f'<odom_topic>/{ns}/odom</odom_topic>'),
        ('<frame_id>odom</frame_id>', f'<frame_id>{ns}/odom</frame_id>'),
        ('<child_frame_id>base_footprint</child_frame_id>',
         f'<child_frame_id>{ns}/base_footprint</child_frame_id>'),
        # Keep DiffDrive's odom->base_footprint transform off the shared /tf.
        ('<tf_topic>/tf</tf_topic>', f'<tf_topic>/{ns}/tf_wheel</tf_topic>'),
        ('<topic>joint_states</topic>', f'<topic>/{ns}/joint_states</topic>'),
    ]
    if strip_sensors:
        text = _SENSOR_RE.sub('', text)
    else:
        pairs += [
            ('<topic>scan</topic>', f'<topic>/{ns}/scan</topic>'),
            ('<gz_frame_id>base_scan</gz_frame_id>',
             f'<gz_frame_id>{ns}/base_scan</gz_frame_id>'),
            ('<topic>imu</topic>', f'<topic>/{ns}/imu</topic>'),
        ]
    for old, new in pairs:
        text = _replace_once(text, old, new)
    return text


def write_namespaced_burger_sdf(stock_path: str, robot_id: int,
                                strip_sensors: bool = True) -> str:
    """Read the stock model, rewrite it, return the path of a temp file."""
    with open(stock_path, newline='') as f:
        text = f.read()
    out = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'robot_{robot_id}_burger_', suffix='.sdf',
        delete=False)
    out.write(namespace_burger_sdf(text, robot_id, strip_sensors))
    out.close()
    return out.name
