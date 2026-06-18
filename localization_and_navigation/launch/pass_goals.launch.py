from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    action_sequence_node = Node(
        package='second_project',
        executable='pass_poses',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    

    # Aggiungiamo entrambe le azioni alla Launch Description
    return LaunchDescription([
        action_sequence_node
    ])