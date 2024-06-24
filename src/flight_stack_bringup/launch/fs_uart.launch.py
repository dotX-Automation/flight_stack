"""
Flight Stack launch file for simulations.

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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Build config files paths
    agent_config = os.path.join(
        get_package_share_directory('micrortps_agent'),
        'config',
        'microRTPS_agent_uart.yaml')
    flight_control_config = os.path.join(
        get_package_share_directory('flight_control'),
        'config',
        'flight_control.yaml')

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf_agent = LaunchConfiguration('cf_agent')
    cf_fc = LaunchConfiguration('cf_fc')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='flight_stack')
    cf_agent_launch_arg = DeclareLaunchArgument(
        'cf_agent',
        default_value=agent_config)
    cf_fc_launch_arg = DeclareLaunchArgument(
        'cf_fc',
        default_value=flight_control_config)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_agent_launch_arg)
    ld.add_action(cf_fc_launch_arg)

    # Create a container
    container = ComposableNodeContainer(
        name='container',
        namespace=ns,
        package='dua_app_management',
        executable='dua_component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='micrortps_agent',
                plugin='MicroRTPSAgent::AgentNode',
                name='microRTPS_agent',
                namespace=ns,
                parameters=[cf_agent],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='flight_control',
                plugin='FlightControl::FlightControlNode',
                name='flight_control',
                namespace=ns,
                parameters=[cf_fc],
                extra_arguments=[{'use_intra_process_comms': True}])])
    ld.add_action(container)

    return ld
