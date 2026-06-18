from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = _truthy(LaunchConfiguration("gui").perform(context))
    use_model_names = _truthy(LaunchConfiguration("use_model_names").perform(context))
    base_watchdog_timeout = float(LaunchConfiguration("base_watchdog_timeout").perform(context))
    is_depth_canonical = _truthy(LaunchConfiguration("is_depth_canonical").perform(context))
    delay_odom_tf_by_one_update = _truthy(LaunchConfiguration("delay_odom_tf_by_one_update").perform(context))

    node_args = []
    if not gui:
        # Keep the classic ROS 1 stageros convention: -g disables the GUI.
        node_args.append("-g")
    if use_model_names:
        node_args.append("-u")
    node_args.append(world)

    return [
        Node(
            package="second_project",
            executable="stageros",
            name="stageros",
            output="screen",
            arguments=node_args,
            parameters=[{
                "base_watchdog_timeout": base_watchdog_timeout,
                "is_depth_canonical": is_depth_canonical,
                "use_model_names": use_model_names,
                "delay_odom_tf_by_one_update": delay_odom_tf_by_one_update,
                "use_sim_time": True,
            }],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('second_project')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_stage_mppi.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'stage_nav2_showcase.rviz')
    default_map = os.path.join(pkg_share, 'map', 'my_map.yaml')
    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    default_world = PathJoinSubstitution([
        FindPackageShare("second_project"), "world", "my_map.world"
    ])

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'namespace': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )



    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map'),
            },
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )


    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'global_frame_id': LaunchConfiguration('map_frame'),
                'set_initial_pose': ParameterValue(LaunchConfiguration('set_initial_pose'), value_type=bool),
                'always_reset_initial_pose': ParameterValue(LaunchConfiguration('set_initial_pose'), value_type=bool),
                'initial_pose': {
                    'x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                    'y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                    'z': 0.0,
                    'yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
                },
            },
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )


    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server', 'amcl'],
        }],
    )


    # 2. Nodo RViz2
    pkg_dir = get_package_share_directory('second_project')
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'stage_nav2_showcase.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[
            {'use_sim_time': True}
        ]
    )


    # action_node = Node(
    #     package='second_project',
    #     executable='publish_action',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True}
    #     ]
    # )
 

    return LaunchDescription([
        #action_node,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the simulator /clock.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically configure and activate Nav2 lifecycle nodes.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('map', default_value=default_map, description='Occupancy map YAML file for map_server. Defaults to maps/stage_maze.yaml inside this package.'),
        DeclareLaunchArgument('scan_topic', default_value='/base_scan', description='Stage LaserScan topic.'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame.'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Global map frame.'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint', description='Robot base frame.'),
        DeclareLaunchArgument('set_initial_pose', default_value='true', description='Let AMCL initialize from initial_x, initial_y, initial_yaw. Set false to use RViz 2D Pose Estimate.'),
        DeclareLaunchArgument('initial_x', default_value='9.0', description='Initial robot x in the map frame.'),
        DeclareLaunchArgument('initial_y', default_value='1.0', description='Initial robot y in the map frame.'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='Initial robot yaw in radians in the map frame.'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params, description='Nav2 parameter file.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level.'),
        map_server,
        amcl,
        lifecycle_manager_localization,
        nav2_navigation,
        start_rviz,
        DeclareLaunchArgument("world", default_value=default_world, description="Stage .world file"),
        DeclareLaunchArgument("gui", default_value="true", description="Start the Stage GUI"),
        DeclareLaunchArgument("use_model_names", default_value="false", description="Use Stage model names as topic/frame prefixes"),
        DeclareLaunchArgument("base_watchdog_timeout", default_value="0.2", description="Seconds before stale cmd_vel commands are zeroed; <=0 disables"),
        DeclareLaunchArgument("is_depth_canonical", default_value="true", description="Use REP-118 32FC1 depth image encoding"),
        DeclareLaunchArgument(
            "delay_odom_tf_by_one_update",
            default_value="true",
            description="Use previous Stage pose for odom/base TF to compensate one-update-delayed LaserScan data",
        ),
        OpaqueFunction(function=_launch_setup),

    ])
