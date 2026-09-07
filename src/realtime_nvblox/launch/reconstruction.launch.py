from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('realtime_nvblox')
    default_config = os.path.join(share, 'config', 'default.yaml')
    default_rviz = os.path.join(share, 'rviz', 'realtime_nvblox.rviz')
    config = LaunchConfiguration('config')
    run_rviz = LaunchConfiguration('run_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('run_rviz', default_value='false'),
        Node(
            package='realtime_nvblox',
            executable='ros_mapper',
            name='realtime_nvblox',
            output='screen',
            parameters=[{'config': config}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            output='screen',
            condition=IfCondition(run_rviz),
        ),
    ])
