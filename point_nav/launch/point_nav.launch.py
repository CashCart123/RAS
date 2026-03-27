from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'config', 'point_nav.defaults.yaml'
    ])
    default_ekf_params_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'config', 'ekf_fusion.yaml'
    ])
    default_urdf_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'urdf', 'point_nav_rover.urdf'
    ])
    default_urdf_xacro_file = PathJoinSubstitution([
        FindPackageShare('point_nav'), 'urdf', 'robot_description.urdf.xacro'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the YAML file with point_nav parameters and goal inputs.'
        ),
        DeclareLaunchArgument(
            'enable_wheel_odom',
            default_value='false',
            description='Run wheel odometry node (/joint_states -> /wheel/odom).'
        ),
        DeclareLaunchArgument(
            'enable_ekf',
            default_value='false',
            description='Run robot_localization EKF fusion (/wheel/odom + camera odom -> /odometry/filtered).'
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=default_ekf_params_file,
            description='Path to the EKF config YAML file.'
        ),
        DeclareLaunchArgument(
            'enable_robot_state_publisher',
            default_value='true',
            description='Publish robot TF tree from URDF via robot_state_publisher.'
        ),
        DeclareLaunchArgument(
            'use_xacro_description',
            default_value='true',
            description='Use Simple_Autonomous xacro robot description (requires xacro and zed_wrapper).'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf_file,
            description='Path to rover URDF file.'
        ),
        DeclareLaunchArgument(
            'urdf_xacro_file',
            default_value=default_urdf_xacro_file,
            description='Path to rover xacro file.'
        ),
        DeclareLaunchArgument(
            'enable_joint_state_from_ticks',
            default_value='false',
            description='Publish /joint_states from /wheel_ticks (Int32MultiArray).'
        ),
        DeclareLaunchArgument(
            'ticks_per_rev',
            default_value='2359296',
            description='Encoder ticks per revolution for wheel tick to joint state conversion.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('urdf_xacro_file')]),
                    value_type=str,
                ),
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('enable_robot_state_publisher'),
                    "' == 'true' and '",
                    LaunchConfiguration('use_xacro_description'),
                    "' == 'true'",
                ])
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', LaunchConfiguration('urdf_file')]),
                    value_type=str,
                ),
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('enable_robot_state_publisher'),
                    "' == 'true' and '",
                    LaunchConfiguration('use_xacro_description'),
                    "' != 'true'",
                ])
            ),
        ),
        Node(
            package='point_nav',
            executable='wheel_joint_state_publisher',
            name='wheel_joint_state_publisher',
            output='screen',
            parameters=[{
                'ticks_per_rev': ParameterValue(LaunchConfiguration('ticks_per_rev'), value_type=float),
            }],
            condition=IfCondition(LaunchConfiguration('enable_joint_state_from_ticks')),
        ),
        Node(
            package='point_nav',
            executable='wheel_odom_node',
            name='wheel_odom',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('enable_wheel_odom')),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_params_file')],
            condition=IfCondition(LaunchConfiguration('enable_ekf')),
        ),
        Node(
            package='point_nav',
            executable='point_nav_node',
            name='point_nav',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'goal_file_path': LaunchConfiguration('params_file'),
                },
            ],
        )
    ])
