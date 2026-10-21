from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('lite_ik_node'), 'config', 'lite_ik_node.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=''),
        DeclareLaunchArgument('srdf_path', default_value=''),
        Node(
            package='lite_ik_node',
            executable='lite_ik_node',
            name='lite_ik_node',
            output='screen',
            parameters=[params, {
                'urdf_path': LaunchConfiguration('urdf_path'),
                'srdf_path': LaunchConfiguration('srdf_path'),
            }],
        ),
    ])
