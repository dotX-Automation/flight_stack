/**
 * Flight Control topic subscription callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 28, 2022
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
 * @brief Checks the battery voltage.
 *
 * @param msg BatteryStatus message to parse.
 */
void FlightControlNode::battery_state_callback(const BatteryStatus::SharedPtr msg)
{
  // Get battery voltage
  double voltage = double(msg->voltage_v);
  if (voltage == 0.0) {
    // Voltage unknown
    return;
  }

  if (!low_battery_ && (voltage <= low_battery_voltage_)) {
    // Check if battery is really low and this wasn't a short load peak
    rclcpp::Time curr_time = clock_.now();

    if (low_battery_timer_ == rclcpp::Time(0, 0, RCL_STEADY_TIME)) {
      // Start the stopwatch
      low_battery_timer_ = curr_time;
      return;
    }

    if ((curr_time - low_battery_timer_) < rclcpp::Duration(5, 0)) {
      // Stopwatch running
      return;
    }

    // Drone should land immediately, warn the user
    low_battery_ = true;
    RCLCPP_WARN(this->get_logger(), "LOW BATTERY (%f V)", voltage);
  } else {
    low_battery_timer_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }
}

/**
 * @brief Prints a log message from PX4.
 *
 * @param msg LogMessage message to parse.
 */
void FlightControlNode::log_message_callback(const LogMessage::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[PX4 (%hhu)] %s",
    msg->severity,
    msg->text.data());
}

/**
 * @brief Sets a new position setpoint received from the position_setpoint topic.
 *
 * @param msg PositionSetpoint message to parse.
 */
void FlightControlNode::position_setpoint_callback(const PositionSetpoint::SharedPtr msg)
{
  change_setpoint(
    Setpoint(
      msg->x_setpoint,
      msg->y_setpoint,
      -abs(msg->z_setpoint),
      msg->yaw_setpoint));
}

/**
 * @brief Gets the latest timestamp from PX4.
 *
 * @param msg PX4Timestamp message to parse.
 */
void FlightControlNode::px4_timestamp_callback(const PX4Timestamp::SharedPtr msg)
{
  fmu_timestamp_.store(msg->timestamp, std::memory_order_release);
}

/**
 * @brief Parses an FMU command ACK from PX4.
 *
 * @param msg VehicleCommandAck message to parse.
 */
void FlightControlNode::vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg)
{
  // If a thread is waiting for this, notify it
  {
    std::lock_guard fmu_cmd_ack_lock(fmu_cmd_ack_lock_);
    bool expected = false;
    if (!fmu_cmd_ack_received_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Might be a retransmission
      return;
    }
    if (msg->result == VehicleCommandAck::VEHICLE_RESULT_ACCEPTED) {
      fmu_cmd_success_.store(true, std::memory_order_release);
    }
  }
  fmu_cmd_ack_cv_.notify_one();

  // Log eventual errors
  switch (msg->result) {
    case VehicleCommandAck::VEHICLE_RESULT_ACCEPTED:
      break;
    case VehicleCommandAck::VEHICLE_RESULT_TEMPORARILY_REJECTED:
      RCLCPP_ERROR(this->get_logger(), "Command temporarily rejected");
      break;
    case VehicleCommandAck::VEHICLE_RESULT_DENIED:
      RCLCPP_ERROR(this->get_logger(), "Command denied");
      break;
    case VehicleCommandAck::VEHICLE_RESULT_UNSUPPORTED:
      RCLCPP_ERROR(this->get_logger(), "Command unsupported");
      break;
    case VehicleCommandAck::VEHICLE_RESULT_FAILED:
      RCLCPP_ERROR(this->get_logger(), "Command failed");
      break;
    case VehicleCommandAck::VEHICLE_RESULT_IN_PROGRESS:
      RCLCPP_ERROR(this->get_logger(), "Command already in progress");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown FMU command result code");
      break;
  }
}

/**
 * @brief Sets a new velocity setpoint received from the velocity_setpoint topic.
 *
 * @param msg VelocitySetpoint message to parse.
 */
void FlightControlNode::velocity_setpoint_callback(const VelocitySetpoint::SharedPtr msg)
{
  change_setpoint(
    Setpoint(
      ControlModes::VELOCITY,
      NAN, NAN, NAN,
      msg->yaw_setpoint,
      msg->vx_setpoint,
      msg->vy_setpoint,
      msg->vz_setpoint,
      msg->vyaw_setpoint));
}

/**
 * @brief Parses a TakeoffStatus update from PX4.
 *
 * @param msg TakeoffStatus message to parse.
 */
void FlightControlNode::takeoff_status_callback(const TakeoffStatus::SharedPtr msg)
{
  uint8_t state = msg->takeoff_state;

  // Drop retransmissions
  if (state == last_takeoff_status_) {
    return;
  }
  last_takeoff_status_ = state;

  // DISARMED
  if (state == TakeoffStatus::TAKEOFF_STATE_DISARMED) {
    // Disarmed or kill switch engaged
    bool expected = true;
    airborne_.compare_exchange_strong(
      expected,
      false,
      std::memory_order_release,
      std::memory_order_acquire);
    armed_.store(false, std::memory_order_release);
    notify_takeoff_status();
    RCLCPP_WARN(this->get_logger(), "DISARMED");
    return;
  }

  // SPOOLUP
  if (state == TakeoffStatus::TAKEOFF_STATE_SPOOLUP) {
    RCLCPP_INFO(this->get_logger(), "SPOOLUP");
    return;
  }

  // READY_FOR_TAKEOFF
  if (state == TakeoffStatus::TAKEOFF_STATE_READY_FOR_TAKEOFF) {
    // Armed or just landed
    bool expected = false;
    if (!armed_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Just landed
      airborne_.store(false, std::memory_order_release);
    }
    notify_takeoff_status();
    RCLCPP_WARN(this->get_logger(), "READY FOR TAKEOFF");
    return;
  }

  // RAMPUP
  if (state == TakeoffStatus::TAKEOFF_STATE_RAMPUP) {
    RCLCPP_INFO(this->get_logger(), "RAMPUP");
    return;
  }

  // FLIGHT
  if (state == TakeoffStatus::TAKEOFF_STATE_FLIGHT) {
    // Armed or taking off (why armed? God knows, PX4 developers surely don't...)
    bool expected = false;
    if (!armed_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Taking off
      airborne_.store(true, std::memory_order_release);
    }
    notify_takeoff_status();
    RCLCPP_WARN(this->get_logger(), "FLIGHT");
    return;
  }
}

/**
 * @brief Pose message filter synchronizer callback.
 *
 * @param local_position_msg VehicleLocalPositionStamped message to parse.
 * @param attitude_msg VehicleAttitudeStamped message to parse.
 */
void FlightControlNode::pose_callback(
  const VehicleLocalPositionStamped::SharedPtr & local_position_msg,
  const VehicleAttitudeStamped::SharedPtr & attitude_msg)
{
  // Drop invalid samples
  if (!(local_position_msg->xy_valid &&
    local_position_msg->z_valid &&
    local_position_msg->v_xy_valid &&
    local_position_msg->v_z_valid))
  {
    return;
  }

  // Compute RPY angles from attitude quaternion
  Eigen::Quaternionf new_attitude = {
    attitude_msg->q[0],
    attitude_msg->q[1],
    attitude_msg->q[2],
    attitude_msg->q[3]};
  Eigen::EulerAnglesXYZf new_rpy(new_attitude);

  Eigen::Vector3f new_position = {
    local_position_msg->x,
    local_position_msg->y,
    local_position_msg->z};
  Eigen::Vector3f new_velocity = {
    local_position_msg->vx,
    local_position_msg->vy,
    local_position_msg->vz};

  DronePose new_pose(new_position, new_velocity, new_attitude, new_rpy);

  // Update internal state
  pthread_spin_lock(&(this->state_lock_));
  drone_pose_ = new_pose;
  last_pose_timestamp_ = clock_.now();
  pthread_spin_unlock(&(this->state_lock_));

  // Set current timestamp for new samples
  rclcpp::Time pose_sample_timestamp = get_clock()->now();

  // Publish NED pose message
  Pose ned_pose_msg{};
  ned_pose_msg.header.set__frame_id(std::string("world"));
  ned_pose_msg.header.set__stamp(pose_sample_timestamp);
  ned_pose_msg.set__x(local_position_msg->x);
  ned_pose_msg.set__y(local_position_msg->y);
  ned_pose_msg.set__z(local_position_msg->z);
  ned_pose_msg.set__vx(local_position_msg->vx);
  ned_pose_msg.set__vy(local_position_msg->vy);
  ned_pose_msg.set__vz(local_position_msg->vz);
  ned_pose_msg.set__attitude_q(attitude_msg->q);
  ned_pose_msg.set__roll(new_rpy.alpha());
  ned_pose_msg.set__pitch(new_rpy.beta());
  ned_pose_msg.set__yaw(new_rpy.gamma());
  pose_pub_->publish(ned_pose_msg);

  // Publish world RViz pose message
  geometry_msgs::msg::PoseStamped world_pose_msg{};
  world_pose_msg.header.set__frame_id(std::string("map"));
  world_pose_msg.header.set__stamp(pose_sample_timestamp);
  world_pose_msg.pose.position.set__x(local_position_msg->x);
  world_pose_msg.pose.position.set__y(local_position_msg->y);
  world_pose_msg.pose.position.set__z(local_position_msg->z);
  world_pose_msg.pose.orientation.set__w(attitude_msg->q[0]);
  world_pose_msg.pose.orientation.set__x(attitude_msg->q[1]);
  world_pose_msg.pose.orientation.set__y(attitude_msg->q[2]);
  world_pose_msg.pose.orientation.set__z(attitude_msg->q[3]);
  rviz_pose_pub_->publish(world_pose_msg);
}

} // namespace FlightControl
