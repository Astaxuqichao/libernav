import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('navflex_bringup')
    bt_dir = get_package_share_directory('navflex_bt_navigator')
    nav2_route_dir = get_package_share_directory('nav2_route')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    chassis_model = LaunchConfiguration('chassis_model')
    params_file_arg = LaunchConfiguration('params_file')
    costmap_params_file_arg = LaunchConfiguration('costmap_params_file')
    rogmap_params_file_arg = LaunchConfiguration('rogmap_params_file')
    bt_params_file = LaunchConfiguration('bt_params_file')
    bt_xml_arg = LaunchConfiguration('default_nav_to_pose_bt_xml')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
    use_bt_navigator = LaunchConfiguration('use_bt_navigator')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')
    graph_filepath = LaunchConfiguration('graph_filepath')
    use_route_server = LaunchConfiguration('use_route_server')
    navigation_type = LaunchConfiguration('navigation_type')

    with_route = use_route_server.perform(context).lower() in ('true', '1', 'yes')
    with_bt_navigator = use_bt_navigator.perform(context).lower() in ('true', '1', 'yes')
    selected_chassis = chassis_model.perform(context).lower()
    selected_navigation_type = navigation_type.perform(context).lower()
    if selected_navigation_type not in ('costmap', 'rogmap', 'both'):
        raise ValueError(
            f'Unsupported navigation_type "{selected_navigation_type}". '
            'Expected "costmap", "rogmap", or "both".')
    legacy_params_file = params_file_arg.perform(context)
    bt_xml = bt_xml_arg.perform(context)

    if not bt_xml:
        bt_xml = os.path.join(
            bt_dir,
            'behavior_trees',
            'navigate_to_pose_rogmap.xml'
            if selected_navigation_type == 'rogmap'
            else 'test_bt_navigator.xml')

    if selected_chassis == 'omni':
        default_costmap_params = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    elif selected_chassis == 'diff':
        default_costmap_params = os.path.join(
            bringup_dir, 'params', 'nav2_params_tb3_diff.yaml')
    else:
        raise ValueError(
            f'Unsupported chassis_model "{selected_chassis}". '
            'Expected "omni" or "diff".')
    default_rogmap_params = os.path.join(bringup_dir, 'params', 'rogmap_params.yaml')

    costmap_params_file = costmap_params_file_arg.perform(context)
    rogmap_params_file = rogmap_params_file_arg.perform(context)

    # Keep params_file as a compatibility override for the selected single
    # backend. In dual mode it retains its historical costmap meaning.
    if selected_navigation_type in ('costmap', 'both'):
        costmap_params_file = (
            costmap_params_file or legacy_params_file or default_costmap_params)
    else:
        costmap_params_file = costmap_params_file or default_costmap_params

    if selected_navigation_type == 'rogmap':
        rogmap_params_file = (
            rogmap_params_file or legacy_params_file or default_rogmap_params)
    else:
        rogmap_params_file = rogmap_params_file or default_rogmap_params

    lifecycle_nodes = []
    if selected_navigation_type in ('costmap', 'both'):
        lifecycle_nodes.append('costmap/navflex_nav')
    if selected_navigation_type in ('rogmap', 'both'):
        lifecycle_nodes.append('rogmap/navflex_nav')
    if with_route:
        lifecycle_nodes.append('route_server')
    if selected_navigation_type in ('costmap', 'both'):
        # velocity_smoother is shared with the robot base, so keep it outside
        # either navigation backend namespace.
        lifecycle_nodes.append('velocity_smoother')
    if with_bt_navigator:
        lifecycle_nodes.append('bt_navigator')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def configured(source_file, root_key):
        return ParameterFile(
            RewrittenYaml(
                source_file=source_file,
                root_key=root_key,
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                },
                convert_types=True),
            allow_substs=True)

    configured_costmap_params = configured(costmap_params_file, 'costmap')
    configured_rogmap_params = configured(rogmap_params_file, 'rogmap')
    configured_velocity_params = ParameterFile(
        RewrittenYaml(
            source_file=costmap_params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True),
        allow_substs=True)

    selected_backend_params = []
    if selected_navigation_type in ('costmap', 'both'):
        selected_backend_params.append(configured_costmap_params)
    if selected_navigation_type in ('rogmap', 'both'):
        selected_backend_params.append(configured_rogmap_params)

    composable_nodes = []
    if selected_navigation_type in ('costmap', 'both'):
        composable_nodes.append(ComposableNode(
            package='navflex_nav',
            plugin='navflex_nav::CostmapNavNode',
            namespace='costmap',
            name='navflex_nav',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'navigation_type': navigation_type},
                configured_costmap_params,
            ],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms},
            ],
            remappings=remappings))
    if selected_navigation_type in ('rogmap', 'both'):
        composable_nodes.append(ComposableNode(
            package='navflex_nav',
            plugin='navflex_nav::RogMapNavNode',
            namespace='rogmap',
            name='navflex_nav',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'navigation_type': navigation_type},
                configured_rogmap_params,
            ],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms},
            ],
            remappings=remappings))

    if selected_navigation_type in ('costmap', 'both'):
        composable_nodes.append(ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_velocity_params],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms},
            ],
            remappings=remappings + [('cmd_vel', '/costmap/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]))

    if with_bt_navigator:
        composable_nodes.append(ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[
                bt_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml,
                },
            ],
            remappings=remappings))

    nodes = [
        Node(
            condition=UnlessCondition(use_composition),
            package='navflex_nav',
            executable='navflex_nav_node',
            name='navflex_nav',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                {'use_sim_time': use_sim_time},
                *selected_backend_params,
                {'navigation_type': navigation_type},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_isolated',
            name=container_name,
            output='screen',
            parameters=selected_backend_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name,
            composable_node_descriptions=composable_nodes),
    ]

    if with_route:
        nodes.append(Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'graph_filepath': graph_filepath},
                {'route_frame': 'map'},
                {'base_frame': 'base_link'},
                {'max_iterations': 0},
                {'min_prune_dist_from_start': 1.0},
                {'min_prune_dist_from_goal': 1.0},
            ]))

    lifecycle_manager_params = [
        {'use_sim_time': use_sim_time},
        {'autostart': autostart},
        {'node_names': lifecycle_nodes},
        {'bond_timeout': 0.0},
        {'bond_heartbeat_period': 0.1},
        {'attempt_respawn_reconnection': True},
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navflex',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=lifecycle_manager_params)

    nodes.extend([
        Node(
            condition=UnlessCondition(use_composition) if with_bt_navigator else IfCondition('false'),
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml,
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=(
                UnlessCondition(use_composition)
                if selected_navigation_type in ('costmap', 'both')
                else IfCondition('false')),
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_velocity_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', '/costmap/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),

        Node(
            condition=UnlessCondition(use_composition),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navflex',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=lifecycle_manager_params),

        TimerAction(
            condition=IfCondition(use_composition),
            period=2.0,
            actions=[lifecycle_manager]),
    ])

    return nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('navflex_bringup')
    bt_dir = get_package_share_directory('navflex_bt_navigator')
    nav2_route_dir = get_package_share_directory('nav2_route')

    default_bt_params_file = os.path.join(bt_dir, 'params', 'navflex_bt_navigator.yaml')
    default_graph = os.path.join(nav2_route_dir, 'graphs', 'sample_graph.geojson')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('chassis_model', default_value='omni',
                              description='Chassis model selecting navigation parameters: omni or diff'),
        DeclareLaunchArgument('navigation_type', default_value='costmap',
                              description='Navigation backend: costmap, rogmap, or both'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Legacy selected-backend parameter override'),
        DeclareLaunchArgument('costmap_params_file', default_value='',
                              description='Costmap backend parameters'),
        DeclareLaunchArgument('rogmap_params_file', default_value='',
                              description='ROGMap backend parameters'),
        DeclareLaunchArgument('bt_params_file', default_value=default_bt_params_file,
                              description='Full path to the bt_navigator parameters file'),
        DeclareLaunchArgument('default_nav_to_pose_bt_xml', default_value='',
                              description='Optional BT XML override. Empty selects by navigation type'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically configure and activate lifecycle nodes'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn navflex_nav if it exits'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='Load NavFlex lifecycle nodes into a component container'),
        DeclareLaunchArgument('use_bt_navigator', default_value='true',
                              description='Launch the standard Nav2 bt_navigator'),
        DeclareLaunchArgument('use_intra_process_comms', default_value='true',
                              description='Use intra-process communication for compatible composable data-path nodes'),
        DeclareLaunchArgument('container_name', default_value='navflex_container',
                              description='Component container name when use_composition is True'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument('graph_filepath', default_value=default_graph,
                              description='Full path to the navigation route graph file'),
        DeclareLaunchArgument('use_route_server', default_value='False',
                              description='Whether to launch nav2_route with the navflex stack'),
        OpaqueFunction(function=launch_setup),
    ])
