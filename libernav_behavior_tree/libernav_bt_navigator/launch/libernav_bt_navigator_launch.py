import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigator_dir = get_package_share_directory('libernav_rogmap_bt_navigator')
    default_bt_xml = os.path.join(
        navigator_dir, 'behavior_trees', 'navigate_to_pose_rogmap.xml')
    default_params_file = os.path.join(
        navigator_dir, 'params', 'rogmap_bt_navigator.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    navigator_node = Node(
        package='libernav_rogmap_bt_navigator',
        executable='rogmap_bt_navigator',
        name='rogmap_bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': bt_xml,
            },
        ],
        ros_arguments=['--disable-rosout-logs'],
        arguments=['--ros-args', '--log-level', log_level])

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_rogmap_bt',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['rogmap_bt_navigator']},
        ],
        ros_arguments=['--disable-rosout-logs'],
        arguments=['--ros-args', '--log-level', log_level])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Activate the independent 3D navigator'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='ROG-Map BT navigator parameter file'),
        DeclareLaunchArgument(
            'default_nav_to_pose_bt_xml', default_value=default_bt_xml,
            description='ROG-Map navigation behavior tree'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Log level'),
        navigator_node,
        lifecycle_manager_node,
    ])
