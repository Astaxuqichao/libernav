from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd_file', description='Absolute path to the terrain PCD file'),
        DeclareLaunchArgument('ground_clearance', default_value='0.5',
                              description='Robot height above PCD terrain in meters'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('ground_search_radius', default_value='0.35'),
        DeclareLaunchArgument('max_ground_step', default_value='0.35'),
        Node(
            package='navflex_3d_pcd_simulator',
            executable='pcd_terrain_simulator',
            name='pcd_terrain_simulator',
            output='screen',
            ros_arguments=['--disable-rosout-logs'],
            parameters=[{
                'pcd_file': LaunchConfiguration('pcd_file'),
                'ground_clearance': LaunchConfiguration('ground_clearance'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'start_yaw': LaunchConfiguration('start_yaw'),
                'ground_search_radius': LaunchConfiguration('ground_search_radius'),
                'max_ground_step': LaunchConfiguration('max_ground_step'),
            }]),
    ])
