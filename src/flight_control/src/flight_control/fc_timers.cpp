/**
 * Flight Control timer callbacks.
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

#define NOOP ((void)0)

#include <flight_control/flight_control.hpp>

namespace flight_control
{

/**
 * @brief Setpoints publishing timer callback.
 *
 * @throws RuntimeError if an impossible thing happens internally, i.e. if there's a bug.
 */
void FlightControlNode::setpoints_timer_callback()
{
  // Check if a setpoint stream is in progress
  if (last_stream_ts_.load(std::memory_order_acquire) != 0ULL) {
    // Check if the stream has timed out
    uint64_t now = get_time_us();
    if ((now - last_stream_ts_.load(std::memory_order_acquire) > setpoint_stream_timeout_us_) &&
      stream_reset_lock_.try_lock())
    {
      last_stream_ts_.store(0ULL, std::memory_order_release);
      stop_drone();
      stream_reset_lock_.unlock();
      RCLCPP_WARN(this->get_logger(), "Setpoint stream timed out, holding position");
    } else {
      return;
    }
  }

  OffboardControlMode control_mode_msg{};
  TrajectorySetpoint setpoint_msg{};
  std::array<float, 3> nans{NAN, NAN, NAN};
  uint64_t timestamp = get_time_us();

  // Get current setpoint in local frame
  setpoint_lock_.lock();
  Setpoint current_setpoint = setpoint_global_to_local(fmu_setpoint_);
  setpoint_lock_.unlock();

  // Fill offboard_control_mode message
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
    // Should never happen, if it does it's a bug
    RCLCPP_FATAL(this->get_logger(), "Invalid OFFBOARD control mode stored");
    throw std::runtime_error("Invalid OFFBOARD control mode stored");
  }

  // Fill trajectory_setpoint message (from map to odom) (from NWU to NED) (vyaw does not change)
  setpoint_msg.set__timestamp(timestamp);
  setpoint_msg.set__acceleration(nans);
  setpoint_msg.set__jerk(nans);
  setpoint_msg.set__thrust(nans);
  if (current_setpoint.control_mode == ControlModes::POSITION) {
    Eigen::Isometry3d setpoint_odom_iso = Eigen::Isometry3d::Identity();
    setpoint_odom_iso.rotate(Eigen::AngleAxisd(current_setpoint.yaw, Eigen::Vector3d::UnitZ()));
    setpoint_odom_iso.pretranslate(
      Eigen::Vector3d(
        current_setpoint.x,
        current_setpoint.y,
        current_setpoint.z));
    Eigen::Quaterniond setpoint_odom_q(setpoint_odom_iso.rotation());

    setpoint_msg.set__x(setpoint_odom_iso.translation().x());
    setpoint_msg.set__y(-setpoint_odom_iso.translation().y());
    setpoint_msg.set__z(-setpoint_odom_iso.translation().z());
    setpoint_msg.set__yaw(-Eigen::EulerAnglesXYZd(setpoint_odom_iso.rotation()).gamma());
    setpoint_msg.set__vx(NAN);
    setpoint_msg.set__vy(NAN);
    setpoint_msg.set__vz(NAN);
    setpoint_msg.set__yawspeed(NAN);

    // Publish the setpoint to RViz for visualization
    PoseStamped rviz_setpoint_msg{};
    rviz_setpoint_msg.header.set__frame_id(odom_frame_);
    rviz_setpoint_msg.header.set__stamp(rclcpp::Time(timestamp * 1000UL));
    rviz_setpoint_msg.pose.position.set__x(setpoint_odom_iso.translation().x());
    rviz_setpoint_msg.pose.position.set__y(setpoint_odom_iso.translation().y());
    rviz_setpoint_msg.pose.position.set__z(setpoint_odom_iso.translation().z());
    rviz_setpoint_msg.pose.orientation.set__w(setpoint_odom_q.w());
    rviz_setpoint_msg.pose.orientation.set__x(setpoint_odom_q.x());
    rviz_setpoint_msg.pose.orientation.set__y(setpoint_odom_q.y());
    rviz_setpoint_msg.pose.orientation.set__z(setpoint_odom_q.z());
    rviz_position_setpoint_pub_->publish(rviz_setpoint_msg);
  } else if (current_setpoint.control_mode == ControlModes::VELOCITY) {
    Eigen::Vector3d v_setpoint_odom(
      current_setpoint.vx,
      current_setpoint.vy,
      current_setpoint.vz);
    double yaw_setpoint_odom = 0.0f;
    if (!std::isnan(current_setpoint.yaw)) {
      yaw_setpoint_odom = current_setpoint.yaw;
    } else {
      yaw_setpoint_odom = NAN;
    }

    setpoint_msg.set__x(NAN);
    setpoint_msg.set__y(NAN);
    setpoint_msg.set__z(NAN);
    setpoint_msg.set__yaw(-yaw_setpoint_odom);
    setpoint_msg.set__vx(v_setpoint_odom.x());
    setpoint_msg.set__vy(-v_setpoint_odom.y());
    setpoint_msg.set__vz(-v_setpoint_odom.z());
    setpoint_msg.set__yawspeed(-current_setpoint.vyaw);
  }

  // Publish messages
  offboard_control_mode_pub_->publish(control_mode_msg);
  trajectory_setpoint_pub_->publish(setpoint_msg);
}

} // namespace flight_control
