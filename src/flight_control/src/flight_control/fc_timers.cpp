/**
 * Flight Control timer callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 26, 2022
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Setpoints publishing timer callback.
 *
 * @throws RuntimeError
 */
void FlightControlNode::setpoints_timer_callback()
{
  OffboardControlMode control_mode_msg{};
  TrajectorySetpoint setpoint_msg{};
  std::array<float, 3> nans{NAN, NAN, NAN};
  uint64_t timestamp = fmu_timestamp_.load(std::memory_order_acquire);

  pthread_spin_lock(&(this->setpoint_lock_));
  Setpoint current_setpoint = fmu_setpoint_;
  pthread_spin_unlock(&(this->setpoint_lock_));

  // Populate offboard_control_mode message
  control_mode_msg.set__timestamp(timestamp);
  control_mode_msg.set__acceleration(false);
  control_mode_msg.set__attitude(false);
  control_mode_msg.set__body_rate(false);
  if (current_setpoint.control_mode == ControlModes::POSITION) {
    control_mode_msg.set__position(true);
    control_mode_msg.set__velocity(false);
  } else if (current_setpoint.control_mode == ControlModes::VELOCITY) {
    control_mode_msg.set__position(false);
    control_mode_msg.set__velocity(true);
  } else {
    // Should never happen
    RCLCPP_FATAL(this->get_logger(), "Invalid OFFBOARD control mode stored");
    throw std::runtime_error("Invalid OFFBOARD control mode stored");
  }

  // Populate trajectory_setpoint message
  setpoint_msg.set__timestamp(timestamp);
  setpoint_msg.set__acceleration(nans);
  setpoint_msg.set__jerk(nans);
  setpoint_msg.set__thrust(nans);
  if (current_setpoint.control_mode == ControlModes::POSITION) {
    setpoint_msg.set__x(current_setpoint.x);
    setpoint_msg.set__y(current_setpoint.y);
    setpoint_msg.set__z(current_setpoint.z);
    setpoint_msg.set__yaw(current_setpoint.yaw);
    setpoint_msg.set__vx(NAN);
    setpoint_msg.set__vy(NAN);
    setpoint_msg.set__vz(NAN);
    setpoint_msg.set__yawspeed(NAN);
  }
  if (current_setpoint.control_mode == ControlModes::VELOCITY) {
    setpoint_msg.set__x(NAN);
    setpoint_msg.set__y(NAN);
    setpoint_msg.set__z(NAN);
    setpoint_msg.set__yaw(current_setpoint.yaw);
    setpoint_msg.set__vx(current_setpoint.vx);
    setpoint_msg.set__vy(current_setpoint.vy);
    setpoint_msg.set__vz(current_setpoint.vz);
    setpoint_msg.set__yawspeed(current_setpoint.vyaw);
  }

  // Publish messages
  offboard_control_mode_pub_->publish(control_mode_msg);
  trajectory_setpoint_pub_->publish(setpoint_msg);
}

} // namespace FlightControl