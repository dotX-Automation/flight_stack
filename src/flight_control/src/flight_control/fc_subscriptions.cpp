/**
 * Flight Control topic subscription callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 28, 2022
 */

#include <flight_control/flight_control.hpp>

namespace FlightControl
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
  if (!check_frame_id_odom(msg->header.frame_id)) {
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
  px4_odom_msg.set__x(msg->pose.pose.position.x);
  px4_odom_msg.set__y(-msg->pose.pose.position.y);
  px4_odom_msg.set__z(-msg->pose.pose.position.z);

  // Set orientation
  px4_odom_msg.set__q(
    {
      float(msg->pose.pose.orientation.w),
      float(msg->pose.pose.orientation.x),
      float(-msg->pose.pose.orientation.y),
      float(-msg->pose.pose.orientation.z)});
  px4_odom_msg.q_offset[0] = NAN;

  // Set pose covariance (the matrix is symmetric, so we only need to copy the upper triangle)
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

  // Set linear velocity
  px4_odom_msg.set__vx(msg->twist.twist.linear.x);
  px4_odom_msg.set__vy(-msg->twist.twist.linear.y);
  px4_odom_msg.set__vz(-msg->twist.twist.linear.z);

  // Set angular velocity
  px4_odom_msg.set__rollspeed(msg->twist.twist.angular.x);
  px4_odom_msg.set__pitchspeed(-msg->twist.twist.angular.y);
  px4_odom_msg.set__yawspeed(-msg->twist.twist.angular.z);

  // Set velocity covariance (the matrix is symmetric, so we only need to copy the upper triangle)
  k = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (j < i) {
        continue;
      }
      double cov = msg->twist.covariance[i * 6 + j];
      px4_odom_msg.velocity_covariance[k++] = float(cov);
    }
  }

  visual_odometry_pub_->publish(px4_odom_msg);
}

/**
 * @brief Sets a new position setpoint received from the position_setpoint topic.
 *
 * @param msg PositionSetpoint message to parse.
 */
void FlightControlNode::position_setpoint_callback(const PositionSetpoint::SharedPtr msg)
{
  if (!check_frame_id(msg->header.frame_id)) {
    return;
  }
  change_setpoint(
    Setpoint(
      msg->position_sp.x,
      msg->position_sp.y,
      abs(msg->position_sp.z),
      msg->yaw_sp));
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
  if (!check_frame_id_drone(msg->header.frame_id)) {
    return;
  }
  if (!operation_lock_.try_lock()) {
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
 * @brief Sets a new velocity setpoint received from the velocity_setpoint topic.
 *
 * @param msg VelocitySetpoint message to parse.
 */
void FlightControlNode::velocity_setpoint_callback(const VelocitySetpoint::SharedPtr msg)
{
  if (!check_frame_id(msg->header.frame_id)) {
    return;
  }
  change_setpoint(
    Setpoint(
      msg->v_sp.x,
      msg->v_sp.y,
      msg->v_sp.z,
      msg->vyaw_sp,
      msg->yaw_sp));
}

/**
 * @brief Forwards a velocity setpoint to PX4.
 *
 * @param msg VelocitySetpoint message to parse.
 */
void FlightControlNode::velocity_stream_callback(const VelocitySetpoint::SharedPtr msg)
{
  if (!check_frame_id(msg->header.frame_id)) {
    return;
  }
  if (!operation_lock_.try_lock()) {
    return;
  }

  last_stream_ts_.store(get_time_us(), std::memory_order_release);

  tf_lock_.lock();
  Eigen::Matrix3d R_odom_map = tf2::transformToEigen(map_to_odom_).rotation().transpose();
  tf_lock_.unlock();

  Eigen::Vector3d v_map(msg->v_sp.x, msg->v_sp.y, msg->v_sp.z);
  Eigen::Vector3d v_odom = R_odom_map * v_map;

  double yaw_sp_odom = 0.0;
  if (!std::isnan(msg->yaw_sp)) {
    Eigen::AngleAxisd yaw_sp_map(msg->yaw_sp, Eigen::Vector3d::UnitZ());
    yaw_sp_odom = Eigen::EulerAnglesXYZd(R_odom_map * yaw_sp_map.toRotationMatrix()).gamma();
  } else {
    yaw_sp_odom = NAN;
  }

  OffboardControlMode control_mode_msg{};
  TrajectorySetpoint setpoint_msg{};
  std::array<float, 3> nans{NAN, NAN, NAN};

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
  control_mode_msg.set__body_rate(false);
  control_mode_msg.set__position(false);
  control_mode_msg.set__velocity(true);

  // Fill trajectory_setpoint message (from NWU to NED) (odom frame) (vyaw does not change)
  setpoint_msg.set__timestamp(timestamp_us);
  setpoint_msg.set__acceleration(nans);
  setpoint_msg.set__jerk(nans);
  setpoint_msg.set__thrust(nans);
  setpoint_msg.set__x(NAN);
  setpoint_msg.set__y(NAN);
  setpoint_msg.set__z(NAN);
  setpoint_msg.set__vx(v_odom.x());
  setpoint_msg.set__vy(-v_odom.y());
  setpoint_msg.set__vz(-v_odom.z());
  setpoint_msg.set__yawspeed(-msg->vyaw_sp);
  setpoint_msg.set__yaw(-yaw_sp_odom);

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
    !check_frame_id_px4(local_position_msg->header.frame_id) ||
    !check_frame_id_px4(attitude_msg->header.frame_id))
  {
    return;
  }

  // Compute data from messages
  Eigen::Vector3d new_position_odom = {
    local_position_msg->x,
    local_position_msg->y,
    local_position_msg->z};
  Eigen::Quaterniond new_attitude_odom = {
    attitude_msg->q[0],
    attitude_msg->q[1],
    attitude_msg->q[2],
    attitude_msg->q[3]};
  Eigen::Vector3d new_velocity_odom = {
    local_position_msg->vx,
    local_position_msg->vy,
    local_position_msg->vz};
  Eigen::Isometry3d new_pose_odom_iso = Eigen::Isometry3d::Identity();
  new_pose_odom_iso.rotate(new_attitude_odom);
  new_pose_odom_iso.pretranslate(new_position_odom);
  tf_lock_.lock();
  Eigen::Isometry3d new_pose_map_iso = tf2::transformToEigen(map_to_odom_) * new_pose_odom_iso;
  Eigen::Matrix3d map_to_odom_rotation = tf2::transformToEigen(map_to_odom_).rotation();
  tf_lock_.unlock();
  Eigen::Quaterniond new_attitude_map = Eigen::Quaterniond(new_pose_map_iso.rotation());
  Eigen::Vector3d new_velocity_map = map_to_odom_rotation * new_velocity_odom;
  Header new_pose_header = local_position_msg->header;
  new_pose_header.set__frame_id("map");

  PoseKit::DynamicPose new_pose(
    new_pose_map_iso.translation(),
    new_attitude_map,
    new_velocity_map,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    new_pose_header);

  PoseKit::KinematicPose new_pose_odom(
    new_position_odom,
    new_attitude_odom,
    new_velocity_odom,
    Eigen::Vector3d::Zero(),
    local_position_msg->header);

  // Update internal state
  state_lock_.lock();
  drone_pose_ = new_pose;
  last_pose_timestamp_ = clock_.now();
  state_lock_.unlock();

  // Set current timestamp for new samples
  rclcpp::Time sample_timestamp = local_position_msg->header.stamp;

  // Publish pose message
  pose_pub_->publish(new_pose.to_euler_pose_stamped());

  // Publish RViz pose message
  rviz_pose_pub_->publish(new_pose.to_pose_stamped());

  // Fill and publish Odometry messages (this is data from PX4's EKF2)
  Odometry odometry_msg{};
  TwistWithCovarianceStamped curr_twist_msg = new_pose_odom.to_twist_with_covariance_stamped();
  curr_twist_msg.twist.twist.angular.set__x(NAN);
  curr_twist_msg.twist.twist.angular.set__y(NAN);
  curr_twist_msg.twist.twist.angular.set__z(NAN);
  odometry_msg.header.set__frame_id(link_namespace_ + "odom");
  odometry_msg.header.set__stamp(sample_timestamp);
  odometry_msg.set__child_frame_id(link_namespace_ + "odom");
  odometry_msg.set__pose(new_pose_odom.to_pose_with_covariance_stamped().pose);
  odometry_msg.set__twist(curr_twist_msg.twist);
  ekf2_odometry_pub_->publish(odometry_msg);
  rviz_ekf2_odometry_pub_->publish(odometry_msg);
}

} // namespace FlightControl
