"""Check deployment launch contracts without starting ROS or hardware."""
import ast
import os
import re
from pathlib import Path
from types import SimpleNamespace as Box

from boat_stack_shim import load_module

ROOT = Path(__file__).resolve().parents[2]


def flight_launch():
    def action(kind):
        return lambda *args, **kwargs: Box(kind=kind, args=args, **kwargs)
    module = load_module(ROOT/'uav_ws/src/uav_bringup/launch/coordinated_autonomy.launch.py', {
        'os': os, 'get_package_share_directory': lambda package: package,
        'LaunchConfiguration': lambda key: key, 'LaunchDescription': lambda actions: actions,
        **{name: action(name) for name in (
            'DeclareLaunchArgument', 'GroupAction', 'IncludeLaunchDescription', 'AnyLaunchDescriptionSource', 'Node', 'SetRemap')},
    })
    return next(a for a in module['generate_launch_description']() if a.kind == 'GroupAction').actions


def test_all_launched_flight_nodes_have_packaged_entry_points():
    for node in (a for a in flight_launch() if a.kind == 'Node'):
        assert node.namespace == '/uav'
        folder = ROOT/'uav_ws/src'/node.package
        tree = ast.parse((folder/'setup.py').read_text())
        entries = [value.value for value in ast.walk(tree)
                   if isinstance(value, ast.Constant) and isinstance(value.value, str) and ' = ' in value.value]
        entry = next(entry for entry in entries if entry.split(' = ')[0] == node.executable)
        module = entry.split(' = ')[1].split(':')[0]
        source = folder.joinpath(*module.split('.')).with_suffix('.py')
        assert source.is_file(), f'Missing module for {entry}'


def test_uav_command_and_state_names_cannot_resolve_to_boat_topics():
    actions = flight_launch()
    remaps = [(a.src, a.dst) for a in actions if a.kind == 'SetRemap']
    def resolve(name):
        for src, dst in remaps:
            if '**' in src:
                match = re.fullmatch(re.escape(src).replace(r'\*\*', '(.*)'), name)
                if match:
                    return dst.replace(r'\1', match.group(1))
            elif name == src:
                return dst
        return name
    for name in ('/mavros/state', '/mavros/setpoint_velocity/cmd_vel', '/control/cmd_vel',
                 '/vehicle/set_autonomy', '/vehicle/arm', '/mission/command', '/mission/start',
                 '/safety/reset_failsafe', '/perception/target', '/por/start'):
        assert resolve(name) == '/uav' + name
    assert resolve('/mavros/mavros/state') == '/uav/mavros/state'
    assert resolve('/uuv/control/cmd_vel') == '/uuv/control/cmd_vel'
    assert resolve('/uav/mapping/buoys') == '/uav/mapping/buoys'
    assert resolve('/tf') == '/uav/tf'
    assert resolve('/tf_static') == '/uav/tf_static'
    mavros = next(a for a in actions if a.kind == 'IncludeLaunchDescription')
    assert dict(mavros.launch_arguments)['namespace'] == '/uav'
