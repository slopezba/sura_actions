import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest


def launch_module():
    path = Path(__file__).parents[1] / 'launch' / 'actions.launch.py'
    spec = importlib.util.spec_from_file_location('actions_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('family,expected', [
    ('surface', {'sura_control_manager_node', 'go_to_pose_usv_lifecycle_action_node',
                 'path_follower_usv_lifecycle_node'}),
    ('underwater', {'sura_control_manager_node', 'surface_action_node',
                    'go_to_depth_lifecycle_action_node', 'go_to_pose_lifecycle_action_node',
                    'path_manager_lifecycle_node'}),
])
def test_family_selection(family, expected):
    module = launch_module()
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['robot_family'] = family
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    module.validate_family(context)
    selected = {
        entity.node_executable
        for entity in description.entities
        if isinstance(entity, Node) and
        (entity.condition is None or entity.condition.evaluate(context))
    }
    assert selected == expected


def test_invalid_family_and_default():
    module = launch_module()
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert context.launch_configurations['robot_family'] == 'underwater'
    context.launch_configurations['robot_family'] = 'unknown'
    with pytest.raises(RuntimeError, match='Unsupported robot_family'):
        module.validate_family(context)
