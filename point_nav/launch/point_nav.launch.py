from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_nav',
            executable='point_nav_node',
            name='point_nav',
            output='screen',
            parameters=[{
                'goal_tolerance': 0.2,
                'max_linear_speed': 0.6,
                'max_angular_speed': 1.0,
                'k_v': 0.6,
                'k_w': 1.5,
                'angle_slowdown_threshold': 0.6
            }]
        )
    ])

