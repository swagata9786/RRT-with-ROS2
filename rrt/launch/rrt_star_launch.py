import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rrt',
            executable='rrt_star_node',
            name='rrt_star_node',
            output='screen',
            parameters=[],
        ),
    ])
