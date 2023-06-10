#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI]
function degrad {
  if [[ $# -ne 1 ]] || [[ $1 -lt -180 ]] || [[ $1 -gt 180 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    degrad ANGLE"
    echo >&2 "ANGLE must be in degrees and in [-180° +180°]"
    return 1
  fi
  local OP
  local FIRST
  local SECOND
  local RES
  OP="scale=6;$1*3.14159265359/180.0"
  RES="$(bc <<<"$OP")"
  FIRST="${RES:0:1}"
  SECOND="${RES:1:1}"
  if [[ $FIRST == "." ]]; then
    RES="0${RES}"
  fi
  if [[ $FIRST == "-" ]] && [[ $SECOND == "." ]]; then
    RES="-0.${RES:2:6}"
  fi
  echo "$RES"
}

# Arms the drone
function arm {
  ros2 action send_goal -f /flight_stack/flight_control/arm dua_interfaces/action/Arm "{}"
}

# Disarms the drone
function disarm {
  ros2 action send_goal -f /flight_stack/flight_control/disarm dua_interfaces/action/Disarm "{}"
}

# Performs the landing procedure
function landing {
  ros2 action send_goal -f /flight_stack/flight_control/landing dua_interfaces/action/Landing "{}"
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
    /flight_stack/flight_control/reach \
    dua_interfaces/action/Reach \
      "{ \
        target_pose: { \
          header: {frame_id: /map}, \
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
    /flight_stack/flight_control/takeoff \
    dua_interfaces/action/Takeoff \
    "{ \
      takeoff_pose: { \
        header: {frame_id: /map}, \
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

  ros2 action send_goal -f /flight_stack/flight_control/turn dua_interfaces/action/Turn "{header: {frame_id: /map}, heading: $(degrad "$1")}"
}

# Calls the FC Reset service
function fc-reset {
  ros2 service call /flight_stack/flight_control/reset std_srvs/srv/Trigger "{}"
}

# Turns on the setpoints stream
function setpoints-on {
  ros2 service call /flight_stack/flight_control/setpoints_switch std_srvs/srv/SetBool "{data: true}"
}

# Turns off the setpoints stream
function setpoints-off {
  ros2 service call /flight_stack/flight_control/setpoints_switch std_srvs/srv/SetBool "{data: false}"
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
    /flight_stack/flight_control/position_setpoint \
    dua_interfaces/msg/PositionSetpoint \
    "{ \
      header: {frame_id: /map}, \
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
      /flight_stack/flight_control/velocity_setpoint \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: /map}, \
        v_sp: { \
          x: $1, \
          y: $2, \
          z: $3 \
        }, \
        yaw_sp: $(degrad "$4") \
      }"
  else
    ros2 topic pub -t 3 \
      /flight_stack/flight_control/velocity_setpoint \
      dua_interfaces/msg/VelocitySetpoint \
      "{ \
        header: {frame_id: /map}, \
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
