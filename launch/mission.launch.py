import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _approval_required():
    """Read Approval_required from mission_params.yaml.

    Read at launch-generation time rather than passed as a launch argument
    because mission_node reads all its configuration from this same YAML; a
    launch argument would be a second source of truth for one setting.
    """
    config_path = os.path.join(
        get_package_share_directory('bv_core'),
        'config',
        'mission_params.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f) or {}
    return bool(config.get('Approval_required', False))


def generate_launch_description():
    mission_node = Node(
        package='bv_core',
        name='mission_node',
        executable='mission_node',
        output='both'
    )

    vision_node = Node(
        package='bv_core',
        executable='vision_node',
        name='vision_node',
        output='both',
    )

    stitching_node = Node(
        package='bv_core',
        executable='stitching_node',
        name='stitching_node',
        output='both',
    )

    filter_node = Node(
        package='bv_core',
        executable='filtering_node',
        name='filtering_node',
        output='both',
    )

    bv_viz_node = Node(
        package='bv_core',
        executable='bv_viz_node',
        name='bv_viz_node',
        output='both',
    )

    actions = [
        mission_node,
        vision_node,
        filter_node,
        stitching_node,
        bv_viz_node,
    ]

    # bv_gcs is only referenced when the gate is enabled, so an unbuilt bv_gcs
    # can never break an autonomous launch.
    if _approval_required():
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('bv_gcs'),
                'launch', 'gcs.launch.py'))
        ))

    return LaunchDescription(actions)
