"""
Flight Control app launch file.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

May 11, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Builds a LaunchDescription for the Flight Control standalone app."""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('flight_control'),
        'config',
        'flight_control.yaml')

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='flight_stack')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='flight_control',
        executable='flight_control_app',
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf])

    ld.add_action(node)

    return ld
