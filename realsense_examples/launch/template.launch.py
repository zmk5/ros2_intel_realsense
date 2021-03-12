"""RealSense Camera Launch Template."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node


def generate_launch_description():
    """Launch template for RealSense camera."""
    rviz_config_dir = os.path.join(
        get_package_share_directory('realsense_ros'), 'config', 'rs_cartographer.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]),
        Node(
            package='realsense_node',
            executable='realsense_node',
            namespace="/t265",
            output='screen',
            parameters=[
                get_package_share_directory('realsense_ros')+'/config/t265.yaml'],
            remappings=[('/t265/camera/odom/sample', '/odom')],
        ),
        Node(
            package='realsense_node',
            executable='realsense_node',
            namespace="/d435",
            output='screen',
            parameters=[
                get_package_share_directory('realsense_ros')+'/config/d435.yaml']
        ),
    ])
