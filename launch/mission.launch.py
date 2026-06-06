import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    human_approval_required = DeclareLaunchArgument(
        'human_approval_required',
        default_value='true',
        description=(
            'When true, insert the bv_gcs approval_node + rosbridge_websocket '
            'between filtering and mission so an operator must approve every '
            'confirmed detection. When false, mission subscribes to the '
            'legacy /global_obj_dets topic and runs fully autonomously.'),
    )
    rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Port for the rosbridge_websocket bridge.',
    )

    approval_required = LaunchConfiguration('human_approval_required')

    # Mission node — listens to /approved_obj_dets when the gate is on,
    # /global_obj_dets when it's off.
    mission_node_gated = Node(
        package='bv_core',
        executable='mission_node',
        name='mission_node',
        output='both',
        parameters=[{'confirmed_topic': '/approved_obj_dets'}],
        condition=IfCondition(approval_required),
    )
    mission_node_auto = Node(
        package='bv_core',
        executable='mission_node',
        name='mission_node',
        output='both',
        parameters=[{'confirmed_topic': '/global_obj_dets'}],
        condition=UnlessCondition(approval_required),
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

    # bv_gcs approval gate + rosbridge — only when human approval is required.
    approval_params = os.path.join(
        get_package_share_directory('bv_gcs'),
        'config', 'approval_params.yaml')
    approval_node = Node(
        package='bv_gcs',
        executable='approval_node',
        name='approval_node',
        output='both',
        parameters=[approval_params],
        condition=IfCondition(approval_required),
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            get_package_share_directory('rosbridge_server'),
            '/launch/rosbridge_websocket_launch.xml']),
        launch_arguments={'port': LaunchConfiguration('rosbridge_port')}.items(),
        condition=IfCondition(approval_required),
    )

    return LaunchDescription([
        human_approval_required,
        rosbridge_port,
        mission_node_gated,
        mission_node_auto,
        vision_node,
        filter_node,
        stitching_node,
        bv_viz_node,
        approval_node,
        rosbridge,
    ])
