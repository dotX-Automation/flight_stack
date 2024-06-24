#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

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

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# shellcheck disable=SC1090

# Source DUA commands
source ~/.dua_submod.sh
source ~/.dua_subtree.sh

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI].
function degrad {
  local angle_in_degrees="$1"
  angle_in_radians=$(python3 -c "import sys, math; angle=float(sys.argv[1]); print(math.radians((angle + 180) % 360 - 180))" "$angle_in_degrees")
  echo "$angle_in_radians"
}

# Routine to update dua-utils.
function utils-update {
  CURR_SHELL=$(ps -p $$ | awk 'NR==2 {print $4}')

  pushd || return
  cd /opt/ros/dua-utils || return
  git pull
  git submodule update --init --recursive
  rm -rf install
  colcon build --merge-install
  rm -rf build log
  source "install/local_setup.$CURR_SHELL"
  popd || return
}

# Arms the drone
function arm {
  ros2 action send_goal -f "$NAMESPACE"/flight_stack/flight_control/arm dua_interfaces/action/Arm "{}"
}

# Disarms the drone
function disarm {
  ros2 action send_goal -f "$NAMESPACE"/flight_stack/flight_control/disarm dua_interfaces/action/Disarm "{}"
}

# Performs the landing procedure
function landing {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    landing MIN_ALTITUDE"
    echo >&2 "MIN_ALTITUDE must be w.r.t. a NWU reference frame."
    return 1
  fi

  ros2 action send_goal -f \
    "$NAMESPACE"/flight_stack/flight_control/landing \
    dua_interfaces/action/Landing \
      "{ \
        minimums: { \
          header: {frame_id: map}, \
          point: {z: $1} \
        } \
      }"
}

# Moves the drone to a target position
function reach {
  # Check input arguments
  if [[ $# -ne 5 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    reach X Y Z YAW RADIUS"
    echo >&2 "XYZ must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°], RADIUS is absolute."
    return 1
  fi

  local yaw_rad
  yaw_rad="$(degrad "$4")"

  ros2 action send_goal -f \
    "$NAMESPACE"/flight_stack/flight_control/reach \
    dua_interfaces/action/Reach \
      "{ \
        target_pose: { \
          header: {frame_id: map}, \
          pose: { \
            position: {x: $1, y: $2, z: $3}, \
            orientation: { \
              w: $(python3 -c "import math; print(math.cos($yaw_rad/2.0))"), \
              x: 0.0, \
              y: 0.0, \
              z: $(python3 -c "import math; print(math.sin($yaw_rad/2.0))") \
            } \
          } \
       },
       reach_radius: $5, \
       stop_at_target: true \
      }"
}

# Performs the takeoff procedure
function takeoff {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    takeoff ALTITUDE"
    echo >&2 "Altitude must be w.r.t. a NWU reference frame."
    return 1
  fi

  ros2 action send_goal -f \
    "$NAMESPACE"/flight_stack/flight_control/takeoff \
    dua_interfaces/action/Takeoff \
    "{ \
      takeoff_pose: { \
        header: {frame_id: map}, \
        pose: { \
           position: {x: 0, y: 0, z: $1}, \
        } \
      } \
    }"
}

# Performs a turn to the desired heading
function turn {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    turn YAW"
    echo >&2 "YAW must be in [-180° +180°]."
    return 1
  fi

  ros2 action send_goal -f "$NAMESPACE"/flight_stack/flight_control/turn dua_interfaces/action/Turn "{header: {frame_id: map}, heading: $(degrad "$1")}"
}

# Calls the FC Reset service
function fc-reset {
  ros2 service call "$NAMESPACE"/flight_stack/flight_control/reset std_srvs/srv/Trigger "{}"
}

# Turns on the setpoints stream
function setpoints-on {
  ros2 service call "$NAMESPACE"/flight_stack/flight_control/setpoints_switch std_srvs/srv/SetBool "{data: true}"
}

# Turns off the setpoints stream
function setpoints-off {
  ros2 service call "$NAMESPACE"/flight_stack/flight_control/setpoints_switch std_srvs/srv/SetBool "{data: false}"
}

# Triggers an FMU reboot
function px4-reboot {
  ros2 service call "$NAMESPACE"/flight_stack/flight_control/px4_reboot std_srvs/srv/Trigger "{}"
}

# Sends a new position setpoint to FC
function position {
  # Check input arguments
  if [[ $# -ne 4 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    position X Y Z YAW"
    echo >&2 "XYZ must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°]."
    return 1
  fi

  ros2 topic pub -t 3 \
    "$NAMESPACE"/flight_stack/flight_control/position_setpoint \
    dua_interfaces/msg/PositionSetpoint \
    "{ \
      header: {frame_id: map}, \
      position_sp: { \
        x: $1, \
        y: $2, \
        z: $3 \
      }, \
      yaw_sp: $(degrad "$4") \
    }"
}

# Sends a new velocity setpoint to FC
function velocity {
  # Check input arguments
  if [[ $# -ne 5 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    velocity VX VY VZ YAW VYAW"
    echo >&2 "VX VY VZ must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°], VYAW must be in [-180°/s +180°/s]."
    echo >&2 "YAW may be NAN to control yaw speed by setting VYAW."
    return 1
  fi

  if [[ $4 != "NAN" ]]; then
    ros2 topic pub -t 3 \
      "$NAMESPACE"/flight_stack/flight_control/velocity_setpoint \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: map}, \
        v_sp: { \
          x: $1, \
          y: $2, \
          z: $3 \
        }, \
        yaw_sp: $(degrad "$4") \
      }"
  else
    ros2 topic pub -t 3 \
      "$NAMESPACE"/flight_stack/flight_control/velocity_setpoint \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: map}, \
        v_sp: { \
          x: $1, \
          y: $2, \
          z: $3 \
        }, \
        yaw_sp: NAN, \
        vyaw_sp: $(degrad "$5") \
      }"
  fi
}

# Sends a new velocity stream setpoint to FC
function velocity-stream {
  # Check input arguments
  if [[ $# -ne 5 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    velocity-stream VX VY VZ YAW VYAW"
    echo >&2 "VX VY VZ must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°], VYAW must be in [-180°/s +180°/s]."
    echo >&2 "YAW may be NAN to control yaw speed by setting VYAW."
    return 1
  fi

  if [[ $4 != "NAN" ]]; then
    ros2 topic pub -r 50 \
      "$NAMESPACE"/flight_stack/flight_control/velocity_stream \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: map}, \
        v_sp: { \
          x: $1, \
          y: $2, \
          z: $3 \
        }, \
        yaw_sp: $(degrad "$4") \
      }"
  else
    ros2 topic pub -r 50 \
      "$NAMESPACE"/flight_stack/flight_control/velocity_stream \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: map}, \
        v_sp: { \
          x: $1, \
          y: $2, \
          z: $3 \
        }, \
        yaw_sp: NAN, \
        vyaw_sp: $(degrad "$5") \
      }"
  fi
}
