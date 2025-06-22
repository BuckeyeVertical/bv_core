import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg = 'bv_core'

    # Mission node
    mission_node = Node(
        package='bv_core',
        name='mission_node',
        executable='mission_node',
        output='both',
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

    return LaunchDescription([
        mission_node,
        vision_node,
        filter_node,
        stitching_node
    ])
