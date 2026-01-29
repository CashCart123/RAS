from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='/home/pat/Simple_Autonomous/robot_description.urdf.xacro',
        description='Absolute path to robot xacro file'
    )

    model = LaunchConfiguration('model')

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )


    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        rviz2,
    ])
