
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='pipeline_config.yaml'),
        DeclareLaunchArgument('output', default_value='./output'),
        DeclareLaunchArgument('bag', default_value=''),

        Node(
            package='bv_core',
            executable='bag_vision_processor',
            name='bag_vision_processor',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'output': LaunchConfiguration('output'),
            }],
            output='screen'
        ),
    ])
