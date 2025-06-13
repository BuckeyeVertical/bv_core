# record.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bag_name = LaunchConfiguration('6-9-25-recording', default='6-9-25-recording')

    return LaunchDescription([
        Node(
            package='ros2bag',
            executable='record',
            name='bag_recorder',
            output='screen',
            arguments=[
                '-o', bag_name,
                '/image_raw',
                '/camera_info',
                #'/mission_state',
                '/mavros/local_position/pose',
                '/mavros/state',
                '/mavros/global_position/global',
                '/mavros/global_position/rel_alt',
                
            ]
        ),
    ])
