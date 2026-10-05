"""Cheap static checks that need neither ROS nor Gazebo."""

import ast
import os
import re
import xml.dom.minidom as minidom

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(*parts):
    with open(os.path.join(ROOT, *parts)) as f:
        return f.read()


def _declared(module):
    return set(re.findall(r"declare_parameter\(\s*'([^']+)'",
                          _read('circular_formation', module)))


def test_every_yaml_parameter_is_declared_by_some_node():
    cfg = yaml.safe_load(_read('config', 'circular_formation.yaml'))
    keys = set(cfg['/**']['ros__parameters'])
    declared = (_declared('formation_controller.py')
                | _declared('formation_monitor.py'))
    assert keys <= declared, f'unused in YAML: {keys - declared}'


def test_launch_overrides_are_declared_by_both_nodes_they_target():
    ctrl = _declared('formation_controller.py')
    mon = _declared('formation_monitor.py')
    shared = {'num_robots', 'spawn_xs', 'spawn_ys', 'spawn_yaws', 'radius',
              'center_mode', 'center_x', 'center_y'}
    assert shared <= ctrl and shared <= mon
    assert {'robot_id', 'omega'} <= ctrl


def test_controller_defaults_match_dataclass_defaults():
    from circular_formation.control_law import FormationParams
    src = _read('circular_formation', 'formation_controller.py')
    for field in ('radius', 'omega', 'k_radial', 'k_angular', 'k_repulse',
                  'd_safe', 'rho_min', 'look_ahead', 'v_max', 'w_max',
                  'comm_range'):
        assert f"declare_parameter('{field}', d.{field})" in src
    cfg = yaml.safe_load(_read('config', 'circular_formation.yaml'))
    y = cfg['/**']['ros__parameters']
    d = FormationParams()
    for field in ('k_radial', 'k_angular', 'k_repulse', 'd_safe', 'rho_min',
                  'look_ahead', 'v_max', 'w_max', 'comm_range'):
        assert y[field] == getattr(d, field), field


def test_entry_points_resolve_to_main_functions():
    setup = _read('setup.py')
    for mod in re.findall(r'circular_formation\.(\w+):main', setup):
        tree = ast.parse(_read('circular_formation', mod + '.py'))
        assert any(isinstance(n, ast.FunctionDef) and n.name == 'main'
                   for n in tree.body), mod


def test_launch_and_nodes_parse():
    for rel in (('launch', 'circular_formation.launch.py'),
                ('circular_formation', 'formation_controller.py'),
                ('circular_formation', 'formation_monitor.py')):
        ast.parse(_read(*rel))


def test_launch_arguments_cover_documented_ones():
    src = _read('launch', 'circular_formation.launch.py')
    declared = set(re.findall(r"declare\('(\w+)'", src))
    assert {'num_robots', 'radius', 'omega', 'center_mode', 'center_x',
            'center_y', 'spawn_layout', 'spawn_scale', 'headless', 'rviz',
            'params_file'} <= declared


def test_world_and_package_xml_well_formed():
    minidom.parseString(_read('worlds', 'formation_world.sdf').encode())
    minidom.parseString(_read('package.xml').encode())
