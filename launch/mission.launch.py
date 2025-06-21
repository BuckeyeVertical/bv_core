import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'bv_core'
    mission_cfg = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'mission_params.yaml'
    )

    vision_cfg = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'vision_params.yaml'
    )

    filtering_cfg = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'filtering_params.yaml'
    )
        
    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('bv_core'),
        'rviz',
        'mission.rviz'
    )

    # Mission node
    mission_node = Node(
        package='bv_core',
        name='mission_node',
        executable='mission',
        parameters=[mission_cfg],
        output='screen',
        shell=True,
    )

    vision_node = Node(
        package='bv_core',
        executable='vision',
        name='vision_node',
        parameters=[vision_cfg],
        output='screen',
        shell=True,
    )

    filter_node = Node(
        package='bv_core',
        executable='filtering',
        name='filtering_node',
        parameters=[filter_node],
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        mission_node,
        vision_node,
        filter_node
    ])
