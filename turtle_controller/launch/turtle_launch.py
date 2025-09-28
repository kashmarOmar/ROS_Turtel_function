from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Start controller node
        Node(
            package='turtle_controller',
            executable='controller_node',
            name='controller'
        )
    ])

