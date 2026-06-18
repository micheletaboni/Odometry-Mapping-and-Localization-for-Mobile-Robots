from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # 1. Nodo Odometer
    odometer_node = Node(
        package='first_project',
        executable='odometer',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    error_node = Node(
        package='first_project',
        executable='tf_error',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # 2. Nodo RViz2
    pkg_dir = get_package_share_directory('first_project')
    rviz_config_dir = os.path.join(pkg_dir, 'rviz2', 'configuration.rviz')
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
    bag_path = os.path.expanduser('~/bags/rosbag2_2026_04_08-16_41_35_fixed')
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen'
    )

    # Aggiungiamo entrambe le azioni alla Launch Description
    return LaunchDescription([
        odometer_node,
        error_node,
        start_rviz,
        play_bag
    ])