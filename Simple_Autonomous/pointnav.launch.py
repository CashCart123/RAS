from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=None,
            executable='/home/pat/Simple_Autonomous/pointnav.py',
            name='waypoint_nav_node',
            output='screen',
            parameters=['/home/pat/Simple_Autonomous/waypoint.yaml'],
        )
    ])
