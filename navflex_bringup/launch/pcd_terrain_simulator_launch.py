import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('navflex_bringup')
    default_pcd = os.path.join(
        bringup_dir, 'maps', 'multilevel_ramp_stairs_0p1m.pcd')
    default_rviz = os.path.join(
        bringup_dir, 'rviz', 'pcd_terrain_3d.rviz')

    parameter_names = [
        ('pcd_file', default_pcd, 'PCD terrain map'),
        ('ground_clearance', '0.5', 'Robot height above the PCD surface'),
        ('ground_grid_resolution', '0.2', 'Spatial index resolution'),
        ('ground_search_radius', '0.35', 'Support-point search radius'),
        ('max_ground_step', '0.35', 'Maximum terrain height change per update'),
        ('start_x', '0.0', 'Initial X position'),
        ('start_y', '0.0', 'Initial Y position'),
        ('start_yaw', '0.0', 'Initial yaw'),
        ('publish_clock', 'true', 'Publish simulation clock'),
    ]

    declarations = [
        DeclareLaunchArgument(name, default_value=value, description=description)
        for name, value, description in parameter_names
    ]

    simulator = Node(
        package='navflex_3d_pcd_simulator',
        executable='pcd_terrain_simulator',
        name='pcd_terrain_simulator',
        output='screen',
        ros_arguments=['--disable-rosout-logs'],
        parameters=[{
            name: LaunchConfiguration(name) for name, _, _ in parameter_names
        }])

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        ros_arguments=['--disable-rosout-logs'],
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    declarations.extend([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz for the PCD terrain'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz configuration file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock in RViz'),
    ])

    return LaunchDescription(declarations + [simulator, rviz])
