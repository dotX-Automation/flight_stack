/**
 * Utility types for the Flight Control node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 6, 2023
 */

#ifndef FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP
#define FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP

#include <cstdint>

namespace FlightControl
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
  ControlModes control_mode = ControlModes::POSITION;
  double x = 0.0; // m
  double y = 0.0; // m
  double z = 0.0; // m
  double yaw = 0.0; // rad
  double vx = 0.0; // m/s
  double vy = 0.0; // m/s
  double vz = 0.0; // m/s
  double vyaw = 0.0; // rad/s

  Setpoint() {}

  Setpoint(
    ControlModes mode,
    double setp_x, double setp_y, double setp_z,
    double setp_yaw,
    double setp_vx, double setp_vy, double setp_vz, double setp_vyaw)
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
  }

  Setpoint(double setp_x, double setp_y, double setp_z, double setp_yaw)
  : control_mode(ControlModes::POSITION),
    x(setp_x),
    y(setp_y),
    z(setp_z),
    yaw(setp_yaw)
  {}

  Setpoint(double vx, double vy, double vz, double vyaw, double yaw)
  : control_mode(ControlModes::VELOCITY),
    yaw(yaw),
    vx(vx),
    vy(vy),
    vz(vz),
    vyaw(vyaw)
  {}
};

}

#endif // FLIGHT_STACK__FLIGHT_CONTROL_TYPES_HPP
