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
        
    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('bv_core'),
        'rviz',
        'mission.rviz'
    )

    # Mission node
    mission_node = Node(
        package='bv_core',
        name='bv_mission',
        executable='mission',
        parameters=[mission_cfg],
        output='screen',
        shell=True,
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    #Uncomment later

    """
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        arguments=[
            '/world/default/model/x500_mono_cam_down_0/'
            'link/camera_link/sensor/imager/image'
            '@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        shell=True
    )

    image_stitching_node = Node(
        package='bv_core',
        executable='image_stitching_node.py',
        output='screen'
    )
    
    detection = Node(
        package='bv_core',
        executable='detection.py',
        output='screen'
    )
    """

    return LaunchDescription([
        mission_node,
        rviz_node,
        # gz_bridge,
        # image_stitching_node,
        # detection
    ])
