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

namespace flight_stack
{

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

  double x = 0.0; // m
  double y = 0.0; // m
  double z = 0.0; // m
  double yaw = 0.0; // rad
  Frame frame = Frame::GLOBAL;

  Setpoint() {}

  Setpoint(
    double setp_x, double setp_y, double setp_z, double setp_yaw,
    Frame setp_frame = Frame::GLOBAL)
  : x(setp_x),
    y(setp_y),
    z(setp_z),
    yaw(setp_yaw),
    frame(setp_frame)
  {}
};

}

#endif // FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP
