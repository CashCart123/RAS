from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'config', 'point_nav.defaults.yaml'
    ])
    default_goal_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'config', 'goal_inputs.defaults.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the YAML file with point_nav parameters.'
        ),
        DeclareLaunchArgument(
            'goal_file_path',
            default_value=default_goal_file,
            description='Path to the YAML file with file-based goal inputs.'
        ),
        Node(
            package='point_nav',
            executable='point_nav_node',
            name='point_nav',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'goal_file_path': LaunchConfiguration('goal_file_path'),
                },
            ],
        )
    ])
