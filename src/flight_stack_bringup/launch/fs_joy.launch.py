"""
Flight Stack joypad launch file for twist-based velocity control.

August 5, 2024
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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name='joy_container',
        namespace='flight_stack',
        package='dua_app_management',
        executable='dua_component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::GameController',
                name='game_controller',
                namespace='flight_stack',
                parameters=[{
                    'device_id': 0,
                    'deadzone': 0.05,
                    'autorepeat_rate': 0.0,
                    'sticky_buttons': False,
                    'coalesce_interval_ms': 1}],
                remappings=[
                    ('/joy', '/flight_stack/joy'),
                    ('/joy/set_feedback', '/flight_stack/joy/set_feedback')]),
            ComposableNode(
                package='teleop_twist_joy',
                plugin='teleop_twist_joy::TeleopTwistJoy',
                name='teleop_twist_joy',
                namespace='flight_stack',
                parameters=[{
                    'require_enable_button': True,
                    'enable_button': 10,
                    'enable_turbo_button': -1,
                    'axis_linear.x': 3,
                    'axis_linear.y': 2,
                    'axis_linear.z': 1,
                    'scale_linear.x': 0.75,
                    'scale_linear.y': 0.75,
                    'scale_linear.z': 0.75,
                    'axis_angular.yaw': 0,
                    'scale_angular.yaw': 0.5,
                    'publish_stamped_twist': False}],
                remappings=[
                    ('/joy', '/flight_stack/joy'),
                    ('/flight_stack/cmd_vel', '/flight_stack/flight_control/cmd_vel')])])
    ld.add_action(container)

    return ld
