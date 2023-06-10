"""
MicroRTPS Agent App launch file for use with FMU via serial port.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

May 23, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the microRTPS Agent App."""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('micrortps_agent'),
        'config',
        'microRTPS_agent_up2.yaml')

    # Create a node action
    node = Node(
        package='micrortps_agent',
        executable='micrortps_agent_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config])
    ld.add_action(node)

    return ld
