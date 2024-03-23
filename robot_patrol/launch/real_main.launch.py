from launch import LaunchDescription
from launch_ros.actions import Node

# For the `right_index_multiplier` parameter use:
#  - 0.75 for simulation
#  - 0.25 for real robot

# For the `mode` parameter use:
#  - simulation
#  - real

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service',
            name='direction_service',
            output='screen',
            parameters=[{'right_index_multiplier': 0.25},],                                       
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service',
            name='patrol_with_service',
            output='screen',
            parameters=[{'mode': 'real'},],                                        
        ),
    ])
