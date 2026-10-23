import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('airsim_offboard')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the node parameter file.'),
        Node(
            package='airsim_offboard',
            executable='offboard_control',
            name='airsim_offboard_control',
            output='screen',
            # The keyboard handler reads stdin; without a TTY it degrades to the
            # line-based prompt fallback.
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
