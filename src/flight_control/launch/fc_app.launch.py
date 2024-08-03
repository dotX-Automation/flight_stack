"""
Flight Control app launch file.

Roberto Masocco <r.masocco@dotxautomation.com>

June 24, 2024
"""

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        parameters=[cf],
        remappings=[
            ('/fmu/battery_state/out', '/flight_stack/microRTPS_agent/fmu/battery_state/out'),
            ('/fmu/log_message/out', '/flight_stack/microRTPS_agent/fmu/log_message/out'),
            ('/fmu/takeoff_status/out', '/flight_stack/microRTPS_agent/fmu/takeoff_status/out'),
            ('/fmu/vehicle_command_ack/out', '/flight_stack/microRTPS_agent/fmu/vehicle_command_ack/out'),
            ('/odometry', '/odometry'),
            ('/fmu/offboard_control_mode/in', '/flight_stack/microRTPS_agent/fmu/offboard_control_mode/in'),
            ('/fmu/trajectory_setpoint/in', '/flight_stack/microRTPS_agent/fmu/trajectory_setpoint/in'),
            ('/fmu/vehicle_command/in', '/flight_stack/microRTPS_agent/fmu/vehicle_command/in'),
            ('/fmu/vehicle_rates_setpoint/in', '/flight_stack/microRTPS_agent/fmu/vehicle_rates_setpoint/in'),
            ('/fmu/vehicle_visual_odometry/in', '/flight_stack/microRTPS_agent/fmu/vehicle_visual_odometry/in'),
            ('/fmu/vehicle_local_position_stamped/out', '/flight_stack/microRTPS_agent/fmu/vehicle_local_position_stamped/out'),
            ('/fmu/vehicle_attitude_stamped/out', '/flight_stack/microRTPS_agent/fmu/vehicle_attitude_stamped/out')])

    ld.add_action(node)

    return ld
