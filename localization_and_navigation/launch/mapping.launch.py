from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def make_mapping_node(context, *args, **kwargs):
    slam_mode = LaunchConfiguration('slam_mode').perform(context).lower()
    if slam_mode == 'sync':
        executable = 'sync_slam_toolbox_node'
    elif slam_mode == 'async':
        executable = 'async_slam_toolbox_node'
    else:
        raise RuntimeError("slam_mode must be either 'async' or 'sync'")

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mode': 'mapping',
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                },
            ],
        )
    ]




def generate_launch_description():
    pkg_share = FindPackageShare('second_project')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'mapper.yaml'])

    # 1. Nodo Convert scan
    convert_scan_node = Node(
        package='second_project',
        executable='convert_scan',
        output='screen',
        parameters=[{
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -3.1415926,  # -M_PI
                'angle_max': 3.1415926,  # M_PI
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.2,
                'range_max': 6.5,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': True
            }],
    )


    # 2. Nodo RViz2
    pkg_dir = get_package_share_directory('second_project')
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'configuration.rviz')
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
    
    # 3. Esecuzione della Bag dal percorso assoluto
    bag_path = os.path.expanduser('~/bags/rosbag2_2026_05_20-11_03_32')
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen'
    )

    # Aggiungiamo entrambe le azioni alla Launch Description
    return LaunchDescription([
        convert_scan_node,
        play_bag,
        start_rviz,
        DeclareLaunchArgument('slam_mode', default_value='async', description='Use async or sync SLAM Toolbox mapping.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the simulator /clock.'),
        DeclareLaunchArgument('scan_topic', default_value='/laser_scan', description='LaserScan topic already published by the simulator.'),
        DeclareLaunchArgument('odom_frame', default_value='UGV_odom', description='Odometry frame.'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame published by SLAM Toolbox.'),
        DeclareLaunchArgument('base_frame', default_value='UGV_base_link', description='Robot base frame used by SLAM Toolbox.'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params, description='SLAM Toolbox parameter file.'),
        OpaqueFunction(function=make_mapping_node),
    ])