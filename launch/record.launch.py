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
                #'/camera/image_raw',
                '/mission_state',
                '/obj_dets',
                '/mavros/local_position/pose',
                '/mavros/global_position/global',
                '/mavros/global_position/rel_alt',
                
            ]
        ),
    ])
