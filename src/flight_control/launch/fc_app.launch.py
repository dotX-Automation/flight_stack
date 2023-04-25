"""
Flight Control app launch file.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

May 11, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the Flight Control app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('flight_control'),
        'config',
        'flight_control.yaml'
    )

    # Create node launch description
    node = Node(
        package='flight_control',
        executable='flight_control',
        exec_name='flight_control_app',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )

    ld.add_action(node)

    return ld
