/**
 * Utility types for the Flight Control node definition.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * June 24, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP
#define FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP

#include <cstdint>

namespace flight_control
{

/**
 * FMU OFFBOARD control modes.
 */
enum class ControlModes : uint8_t
{
  POSITION,
  VELOCITY
};

/**
 * FMU setpoint.
 */
struct Setpoint
{
  enum class Frame : uint8_t
  {
    LOCAL,
    GLOBAL
  };

  ControlModes control_mode = ControlModes::POSITION;
  double x = 0.0; // m
  double y = 0.0; // m
  double z = 0.0; // m
  double yaw = 0.0; // rad
  double vx = 0.0; // m/s
  double vy = 0.0; // m/s
  double vz = 0.0; // m/s
  double vyaw = 0.0; // rad/s
  Frame frame = Frame::GLOBAL;

  Setpoint() {}

  Setpoint(
    ControlModes mode,
    double setp_x, double setp_y, double setp_z,
    double setp_yaw,
    double setp_vx, double setp_vy, double setp_vz, double setp_vyaw,
    Frame setp_frame = Frame::GLOBAL)
  {
    control_mode = mode;
    x = setp_x;
    y = setp_y;
    z = setp_z;
    yaw = setp_yaw;
    vx = setp_vx;
    vy = setp_vy;
    vz = setp_vz;
    vyaw = setp_vyaw;
    frame = setp_frame;
  }

  Setpoint(
    double setp_x, double setp_y, double setp_z, double setp_yaw,
    Frame setp_frame = Frame::GLOBAL)
  : control_mode(ControlModes::POSITION),
    x(setp_x),
    y(setp_y),
    z(setp_z),
    yaw(setp_yaw),
    frame(setp_frame)
  {}

  Setpoint(double vx, double vy, double vz, double vyaw, double yaw,
    Frame setp_frame = Frame::GLOBAL)
  : control_mode(ControlModes::VELOCITY),
    yaw(yaw),
    vx(vx),
    vy(vy),
    vz(vz),
    vyaw(vyaw),
    frame(setp_frame)
  {}
};

}

#endif // FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP
