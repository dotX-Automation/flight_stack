/**
 * Flight Control topic subscription callbacks.
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

#include <flight_control/flight_control.hpp>

namespace flight_stack
{

/**
 * @brief Checks the battery voltage.
 *
 * @param msg BatteryStatus message to parse.
 */
void FlightControlNode::battery_state_callback(const BatteryState::SharedPtr msg)
{
  // Get battery voltage
  double voltage = double(msg->voltage);
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
    RCLCPP_WARN(this->get_logger(), "LOW BATTERY (%.2f V)", voltage);
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
 * @brief Forwards a new odometry sample to PX4 (from NWU to NED).
 *
 * @param msg Odometry message to parse.
 */
void FlightControlNode::odometry_callback(const Odometry::SharedPtr msg)
{
  if (!check_frame_id_local(msg->header.frame_id)) {
    return;
  }

  VehicleVisualOdometry px4_odom_msg{};

  // Set timestamps and local reference frames
  px4_odom_msg.set__timestamp(
    msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL);
  px4_odom_msg.set__timestamp_sample(px4_odom_msg.timestamp);
  px4_odom_msg.set__local_frame(VehicleVisualOdometry::LOCAL_FRAME_NED);
  px4_odom_msg.set__velocity_frame(VehicleVisualOdometry::BODY_FRAME_FRD);

  // Set position
  if (data_to_px4_[0]) {
    px4_odom_msg.set__x(msg->pose.pose.position.x);
    px4_odom_msg.set__y(-msg->pose.pose.position.y);
    px4_odom_msg.set__z(-msg->pose.pose.position.z);
  } else {
    px4_odom_msg.set__x(NAN);
    px4_odom_msg.set__y(NAN);
    px4_odom_msg.set__z(NAN);
  }

  // Set orientation
  if (data_to_px4_[1]) {
    px4_odom_msg.set__q(
      {
        float(msg->pose.pose.orientation.w),
        float(msg->pose.pose.orientation.x),
        float(-msg->pose.pose.orientation.y),
        float(-msg->pose.pose.orientation.z)});
  } else {
    px4_odom_msg.set__q({NAN, NAN, NAN, NAN});
  }
  px4_odom_msg.q_offset[0] = NAN;

  // Set pose covariance (the matrix is symmetric, so we only need to copy the upper triangle)
  if (data_to_px4_[2]) {
    int k = 0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        if (j < i) {
          continue;
        }
        double cov = msg->pose.covariance[i * 6 + j];
        px4_odom_msg.pose_covariance[k++] = float(cov);
      }
    }
  } else {
    px4_odom_msg.pose_covariance[0] = NAN;
    px4_odom_msg.pose_covariance[15] = NAN;
  }

  // Set linear velocity
  if (data_to_px4_[3]) {
    px4_odom_msg.set__vx(msg->twist.twist.linear.x);
    px4_odom_msg.set__vy(-msg->twist.twist.linear.y);
    px4_odom_msg.set__vz(-msg->twist.twist.linear.z);
  } else {
    px4_odom_msg.set__vx(NAN);
    px4_odom_msg.set__vy(NAN);
    px4_odom_msg.set__vz(NAN);
  }

  // Set angular velocity
  if (data_to_px4_[4]) {
    px4_odom_msg.set__rollspeed(msg->twist.twist.angular.x);
    px4_odom_msg.set__pitchspeed(-msg->twist.twist.angular.y);
    px4_odom_msg.set__yawspeed(-msg->twist.twist.angular.z);
  } else {
    px4_odom_msg.set__rollspeed(NAN);
    px4_odom_msg.set__pitchspeed(NAN);
    px4_odom_msg.set__yawspeed(NAN);
  }

  // Set velocity covariance (the matrix is symmetric, so we only need to copy the upper triangle)
  if (data_to_px4_[5]) {
    int k = 0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        if (j < i) {
          continue;
        }
        double cov = msg->twist.covariance[i * 6 + j];
        px4_odom_msg.velocity_covariance[k++] = float(cov);
      }
    }
  } else {
    px4_odom_msg.velocity_covariance[0] = NAN;
    px4_odom_msg.velocity_covariance[15] = NAN;
  }

  visual_odometry_pub_->publish(px4_odom_msg);
}

/**
 * @brief Sets a new position setpoint received from the position_setpoint topic.
 *
 * @param msg PoseStamped message to parse.
 */
void FlightControlNode::position_setpoint_callback(const PoseStamped::SharedPtr msg)
{
  if (!check_frame_id_global(msg->header.frame_id) ||
    !(armed_.load(std::memory_order_acquire) && airborne_.load(std::memory_order_acquire)) ||
    (!operation_lock_.try_lock()))
  {
    return;
  }

  Eigen::Quaterniond q(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);
  Eigen::EulerAnglesXYZd rpy_q(q.toRotationMatrix());

  change_setpoint(
    Setpoint(
      msg->pose.position.x,
      msg->pose.position.y,
      abs(msg->pose.position.z),
      rpy_q.gamma(),
      Setpoint::Frame::GLOBAL),
    update_setpoint_);

  operation_lock_.unlock();
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

  // Log errors
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
 * @brief Forwards a rates setpoint to PX4.
 *
 * @param msg RatesSetpoint message to parse.
 */
void FlightControlNode::rates_stream_callback(const RatesSetpoint::SharedPtr msg)
{
  if (!check_frame_id_body(msg->header.frame_id) ||
    !(armed_.load(std::memory_order_acquire) && airborne_.load(std::memory_order_acquire)) ||
    (!operation_lock_.try_lock()))
  {
    return;
  }

  last_stream_ts_.store(get_time_us(), std::memory_order_release);

  OffboardControlMode control_mode_msg{};
  VehicleRatesSetpoint setpoint_msg{};

  // Get timestamp from current message, or from current time (to enable CLI control)
  uint64_t timestamp_us = 0UL;
  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    timestamp_us = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
  } else {
    timestamp_us = get_time_us();
  }

  // Fill offboard_control_mode message
  control_mode_msg.set__timestamp(timestamp_us);
  control_mode_msg.set__acceleration(false);
  control_mode_msg.set__attitude(false);
  control_mode_msg.set__body_rate(true);
  control_mode_msg.set__position(false);
  control_mode_msg.set__velocity(false);

  // Fill vehicle_rates_setpoint message (convert from NWU to NED)
  setpoint_msg.set__timestamp(timestamp_us);
  setpoint_msg.set__roll(msg->roll_rate);
  setpoint_msg.set__pitch(-msg->pitch_rate);
  setpoint_msg.set__yaw(-msg->yaw_rate);
  setpoint_msg.thrust_body[0] = 0;
  setpoint_msg.thrust_body[1] = 0;
  setpoint_msg.thrust_body[2] = -msg->thrust;

  // Publish messages
  offboard_control_mode_pub_->publish(control_mode_msg);
  vehicle_rates_setpoint_pub_->publish(setpoint_msg);

  operation_lock_.unlock();
}

/**
 * @brief Forwards a velocity setpoint to PX4.
 *
 * @param msg Twist message to parse.
 */
void FlightControlNode::velocity_setpoint_callback(const Twist::SharedPtr msg)
{
  if (!(armed_.load(std::memory_order_acquire) && airborne_.load(std::memory_order_acquire)) ||
    (!operation_lock_.try_lock()))
  {
    return;
  }

  last_stream_ts_.store(get_time_us(), std::memory_order_release);

  // Get the latest local -> body rotation matrix
  TransformStamped local_to_body{};
  rclcpp::Time tf_time{};
  while (true) {
    try {
      local_to_body = tf_buffer_->lookupTransform(
        local_frame_,
        body_frame_,
        tf_time,
        tf2::durationFromSec(tf2_timeout_));
      break;
    } catch (const tf2::ExtrapolationException & e) {
      // Just retry
      continue;
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "FlightControlNode::velocity_stream_callback: TF exception: %s",
        e.what());
      return;
    }
  }
  Eigen::Matrix3d R_local_body = tf2::transformToEigen(local_to_body).rotation();

  Eigen::Vector3d v_linear_body(msg->linear.x, msg->linear.y, msg->linear.z);
  Eigen::Vector3d v_linear_local = R_local_body * v_linear_body;

  OffboardControlMode control_mode_msg{};
  TrajectorySetpoint setpoint_msg{};
  std::array<float, 3> nans{NAN, NAN, NAN};

  // Saturate velocity along X axis
  double v_x = fmin(
    fmax(
      v_linear_local.x(),
      -velocity_control_vhorz_max_),
    velocity_control_vhorz_max_);

  // Saturate velocity along Y axis
  double v_y = fmin(
    fmax(
      v_linear_local.y(),
      -velocity_control_vhorz_max_),
    velocity_control_vhorz_max_);

  // Saturate velocity along Z axis
  double v_z = fmin(
    fmax(
      v_linear_local.z(),
      -velocity_control_vvert_max_),
    velocity_control_vvert_max_);

  // Saturate angular velocity
  double v_yaw = fmin(
    fmax(
      msg->angular.z,
      -velocity_control_vyaw_max_),
    velocity_control_vyaw_max_);

  // Get timestamp from current time
  uint64_t timestamp_us = get_time_us();

  // Fill offboard_control_mode message
  control_mode_msg.set__timestamp(timestamp_us);
  control_mode_msg.set__acceleration(false);
  control_mode_msg.set__attitude(false);
  control_mode_msg.set__body_rate(false);
  control_mode_msg.set__position(false);
  control_mode_msg.set__velocity(true);

  // Fill trajectory_setpoint message (from NWU to NED) (local frame) (vyaw does not change)
  setpoint_msg.set__timestamp(timestamp_us);
  setpoint_msg.set__acceleration(nans);
  setpoint_msg.set__jerk(nans);
  setpoint_msg.set__thrust(nans);
  setpoint_msg.set__x(NAN);
  setpoint_msg.set__y(NAN);
  setpoint_msg.set__z(NAN);
  setpoint_msg.set__vx(v_x);
  setpoint_msg.set__vy(-v_y);
  setpoint_msg.set__vz(-v_z);
  setpoint_msg.set__yawspeed(-v_yaw);
  setpoint_msg.set__yaw(NAN);

  // Publish messages
  offboard_control_mode_pub_->publish(control_mode_msg);
  trajectory_setpoint_pub_->publish(setpoint_msg);

  operation_lock_.unlock();
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
    local_position_msg->v_z_valid) ||
    !check_frame_id_local(local_position_msg->header.frame_id) ||
    !check_frame_id_local(attitude_msg->header.frame_id))
  {
    return;
  }

  // Get the latest global -> local transform
  TransformStamped global_to_local{};
  rclcpp::Time tf_time = local_position_msg->header.stamp;
  while (true) {
    try {
      global_to_local = tf_buffer_->lookupTransform(
        global_frame_,
        local_frame_,
        tf_time,
        tf2::durationFromSec(tf2_timeout_));
      break;
    } catch (const tf2::ExtrapolationException & e) {
      // Just get the latest
      tf_time = rclcpp::Time{};
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "FlightControlNode::pose_callback: TF exception: %s",
        e.what());
      return;
    }
  }

  // Compute data from messages
  Eigen::Vector3d new_position_local = {
    local_position_msg->x,
    local_position_msg->y,
    local_position_msg->z};
  Eigen::Quaterniond new_attitude_local = {
    attitude_msg->q[0],
    attitude_msg->q[1],
    attitude_msg->q[2],
    attitude_msg->q[3]};
  Eigen::Vector3d new_velocity_local = {
    local_position_msg->vx,
    local_position_msg->vy,
    local_position_msg->vz};
  Eigen::Isometry3d new_pose_local_iso = Eigen::Isometry3d::Identity();
  new_pose_local_iso.rotate(new_attitude_local);
  new_pose_local_iso.pretranslate(new_position_local);
  Eigen::Isometry3d new_pose_global_iso = tf2::transformToEigen(global_to_local) *
    new_pose_local_iso;
  Eigen::Matrix3d global_to_local_rotation = tf2::transformToEigen(global_to_local).rotation();
  Eigen::Quaterniond new_attitude_global = Eigen::Quaterniond(new_pose_global_iso.rotation());
  Eigen::Vector3d new_velocity_global = global_to_local_rotation * new_velocity_local;
  Header new_pose_header = local_position_msg->header;
  new_pose_header.set__frame_id(global_frame_);

  pose_kit::DynamicPose new_pose(
    new_pose_global_iso.translation(),
    new_attitude_global,
    new_velocity_global,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    new_pose_header);

  pose_kit::KinematicPose new_pose_local(
    new_position_local,
    new_attitude_local,
    new_velocity_local,
    Eigen::Vector3d::Zero(),
    local_position_msg->header);

  // Update internal state
  state_lock_.lock();
  drone_pose_ = new_pose;
  drone_pose_local_ = new_pose_local;
  last_pose_timestamp_ = clock_.now();
  state_lock_.unlock();

  // Set current timestamp for new samples
  rclcpp::Time sample_timestamp = local_position_msg->header.stamp;

  // Publish pose message
  ekf2_pose_pub_->publish(new_pose.to_pose_stamped());

  // Fill and publish Odometry messages (this is data from PX4's EKF2)
  Odometry odometry_msg{};
  TwistWithCovarianceStamped curr_twist_msg = new_pose_local.to_twist_with_covariance_stamped();
  curr_twist_msg.twist.twist.angular.set__x(NAN);
  curr_twist_msg.twist.twist.angular.set__y(NAN);
  curr_twist_msg.twist.twist.angular.set__z(NAN);
  odometry_msg.header.set__frame_id(local_frame_);
  odometry_msg.header.set__stamp(sample_timestamp);
  odometry_msg.set__child_frame_id(local_frame_);
  odometry_msg.set__pose(new_pose_local.to_pose_with_covariance_stamped().pose);
  odometry_msg.set__twist(curr_twist_msg.twist);
  ekf2_odometry_pub_->publish(odometry_msg);

  // Publish tf with EKF2 data
  if (publish_tf_) {
    TransformStamped ekf2_tf{};
    ekf2_tf.header.set__frame_id(local_frame_);
    ekf2_tf.header.set__stamp(sample_timestamp);
    ekf2_tf.child_frame_id = body_frame_;

    ekf2_tf.transform.translation.set__x(new_pose_local.get_position().x());
    ekf2_tf.transform.translation.set__y(new_pose_local.get_position().y());
    ekf2_tf.transform.translation.set__z(new_pose_local.get_position().z());

    ekf2_tf.transform.rotation.set__w(new_pose_local.get_attitude().w());
    ekf2_tf.transform.rotation.set__x(new_pose_local.get_attitude().x());
    ekf2_tf.transform.rotation.set__y(new_pose_local.get_attitude().y());
    ekf2_tf.transform.rotation.set__z(new_pose_local.get_attitude().z());

    tf_broadcaster_->sendTransform(ekf2_tf);
  }
}

} // namespace flight_stack
