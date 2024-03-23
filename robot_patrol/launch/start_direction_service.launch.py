from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service',
            name='direction_service',
            output='screen',
            parameters=[{'right_index_multiplier': 0.25}],  # Ensure this matches your expected parameter
        ),
    ])
