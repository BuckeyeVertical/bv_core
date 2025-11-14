import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg = 'bv_core'
    mission_config = LaunchConfiguration('mission_config')
    default_mission_config = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'mission_params.yaml'
    )

    declare_mission_config = DeclareLaunchArgument(
        'mission_config',
        default_value=default_mission_config,
        description='Path to mission parameter YAML file.'
    )

    # Mission node
    mission_node = Node(
        package='bv_core',
        name='mission_node',
        executable='mission_node',
        output='both',
        parameters=[{'mission_config': mission_config}]
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

    return LaunchDescription([
        declare_mission_config,
        mission_node,
        vision_node,
        filter_node,
        stitching_node,
        bv_viz_node,
    ])
