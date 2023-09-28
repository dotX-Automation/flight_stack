/**
 * Flight Control module auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 24, 2022
 */

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Routine to activate setpoints publishing timer.
 */
void FlightControlNode::activate_setpoints_timer()
{
  setpoints_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(setpoints_period_),
    std::bind(
      &FlightControlNode::setpoints_timer_callback,
      this),
    setpoints_timer_cgroup_);
  std::this_thread::sleep_for(std::chrono::seconds(1)); // Let PX4 adapt to rate, see docs
  RCLCPP_WARN(this->get_logger(), "Setpoints timer ON");
}

/**
 * @brief Routine to deactivate setpoints publishing timer.
 */
void FlightControlNode::deactivate_setpoints_timer()
{
  setpoints_timer_->cancel();
  setpoints_timer_.reset();
  RCLCPP_WARN(this->get_logger(), "Setpoints timer OFF");
}

/**
 * @brief Routine to update target setpoint.
 *
 * @param new_setpoint New target setpoint to store.
 *
 * @return Update performed or not.
 */
bool FlightControlNode::change_setpoint(const Setpoint & new_setpoint, bool to_update)
{
  // Check control mode
  if ((new_setpoint.control_mode != ControlModes::POSITION) &&
    (new_setpoint.control_mode != ControlModes::VELOCITY))
  {
    // Should never happen
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: unknown control mode");
    return false;
  }
  // Check yaw angle: must be in [-PI +PI]
  if (!std::isnan(new_setpoint.yaw) &&
    ((new_setpoint.yaw < -M_PI) || (new_setpoint.yaw > M_PI)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: yaw out of range");
    return false;
  }
  // Check POSITION: target coordinates
  if ((new_setpoint.control_mode == ControlModes::POSITION) &&
    (std::isnan(new_setpoint.x) || std::isnan(new_setpoint.y) || std::isnan(new_setpoint.z) ||
    (new_setpoint.z < 0.0) ||
    std::isnan(new_setpoint.yaw)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid position coordinates");
    return false;
  }
  // Check VELOCITY: linear velocities
  if ((new_setpoint.control_mode == ControlModes::VELOCITY) &&
    (std::isnan(new_setpoint.vx) || std::isnan(new_setpoint.vy) || std::isnan(new_setpoint.vz)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid linear velocities");
    return false;
  }
  // Check VELOCITY: yaw configuration
  if ((new_setpoint.control_mode == ControlModes::VELOCITY) &&
    (std::isnan(new_setpoint.yaw) && std::isnan(new_setpoint.vyaw)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid yaw configuration");
    return false;
  }

  // Check if setpoint needs to be "updated", i.e., continuously transformed from global to local frame
  Setpoint new_setpoint_local = new_setpoint;
  if (to_update) {
    // We just require the setpoint to be in the global frame
    if (new_setpoint_local.frame != Setpoint::Frame::GLOBAL) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid new setpoint: should be updated but is not in global frame");
      return false;
    }
  } else {
    // We must convert it now, just this time
    new_setpoint_local = setpoint_global_to_local(new_setpoint_local);
  }

  // Update stored setpoint
  last_stream_ts_.store(0ULL, std::memory_order_release);
  {
    std::unique_lock setpoint_lk(setpoint_lock_);
    fmu_setpoint_ = new_setpoint_local;
  }

  return true;
}

/**
 * @brief Transforms a setpoint from the global frame to the local frame.
 *
 * @param global_setpoint Setpoint in the global frame.
 * @return Setpoint in the local frame.
 *
 * @throws RuntimeError if setpoint is invalid.
 */
Setpoint FlightControlNode::setpoint_global_to_local(const Setpoint & global_setpoint)
{
  // Consistency check
  if (global_setpoint.frame == Setpoint::Frame::LOCAL) {
    return global_setpoint;
  }

  // Get the latest odom -> map transform
  TransformStamped map_to_odom{};
  rclcpp::Time tf_time = this->get_clock()->now();
  while (true) {
    try {
      map_to_odom = tf_buffer_->lookupTransform(
        map_frame_,
        odom_frame_,
        tf_time,
        tf2::durationFromSec(tf2_timeout_));
      break;
    } catch (const tf2::ExtrapolationException & e) {
      // Just get the latest
      tf_time = rclcpp::Time{};
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "FlightControlNode::setpoint_global_to_local: TF exception: %s",
        e.what());
      // It is not possible to return a "dummy" value here, since the caller will use it to
      // control the drone; we have to keep trying
    }
  }
  Eigen::Isometry3d odom_map_iso = tf2::transformToEigen(map_to_odom).inverse();

  // Transform the setpoint into the local frame
  if (global_setpoint.control_mode == ControlModes::POSITION) {
    Eigen::Isometry3d global_setpoint_iso = Eigen::Isometry3d::Identity();
    global_setpoint_iso.rotate(Eigen::AngleAxisd(global_setpoint.yaw, Eigen::Vector3d::UnitZ()));
    global_setpoint_iso.pretranslate(
      Eigen::Vector3d(
        global_setpoint.x,
        global_setpoint.y,
        global_setpoint.z));
    Eigen::Isometry3d local_setpoint_iso = odom_map_iso * global_setpoint_iso;
    Eigen::EulerAnglesXYZd local_setpoint_rpy(local_setpoint_iso.rotation());

    return Setpoint(
      local_setpoint_iso.translation().x(),
      local_setpoint_iso.translation().y(),
      local_setpoint_iso.translation().z(),
      local_setpoint_rpy.gamma(),
      Setpoint::Frame::LOCAL);
  } else if (global_setpoint.control_mode == ControlModes::VELOCITY) {
    Eigen::Vector3d global_setpoint_v(global_setpoint.vx, global_setpoint.vy, global_setpoint.vz);
    Eigen::AngleAxisd global_yaw(global_setpoint.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d local_setpoint_v = odom_map_iso * global_setpoint_v;
    Eigen::AngleAxisd local_yaw(odom_map_iso.rotation() * global_yaw.toRotationMatrix());

    return Setpoint(
      local_setpoint_v.x(),
      local_setpoint_v.y(),
      local_setpoint_v.z(),
      global_setpoint.vyaw,
      local_yaw.angle(),
      Setpoint::Frame::LOCAL);
  } else {
    // Should never happen, if it does it's a bug
    RCLCPP_FATAL(
      this->get_logger(),
      "FlightControlNode::setpoint_global_to_local: Invalid OFFBOARD control mode stored");
    throw std::runtime_error(
            "FlightControlNode::setpoint_global_to_local: Invalid OFFBOARD control mode stored");
  }
}

/**
 * @brief Routine to send a command to PX4 over the vehicle_command topic.
 *
 * @param cmd Command to publish.
 * @param p1 Command parameter 1.
 * @param p2 Command parameter 2.
 * @param p3 Command parameter 3.
 * @param p4 Command parameter 4.
 * @param p5 Command parameter 5.
 * @param p6 Command parameter 6.
 * @param p7 Command parameter 7.
 *
 * @return Command publishing operation result.
 */
bool FlightControlNode::send_fmu_command(
  uint16_t cmd,
  float p1,
  float p2,
  float p3,
  float p4,
  float p5,
  float p6,
  float p7)
{
  VehicleCommand msg{};
  msg.set__timestamp(get_time_us());

  // Set MAVLink command code and parameters
  msg.set__command(cmd);
  msg.set__param1(p1);
  msg.set__param2(p2);
  msg.set__param3(p3);
  msg.set__param4(p4);
  msg.set__param5(p5);
  msg.set__param6(p6);
  msg.set__param7(p7);

  // Set uORB-related fields to defaults
  msg.set__target_system(1);
  msg.set__target_component(1);
  msg.set__source_system(1);
  msg.set__source_component(1);
  msg.set__from_external(true);
  msg.set__confirmation(0);

  // Attempt to publish the command and get the ACK back
  int64_t max_attempts = fmu_command_attempts_;
  int64_t command_timeout = fmu_command_timeout_;
  for (int64_t attempt = 0; attempt < max_attempts; attempt++) {
    std::unique_lock fmu_cmd_ack_lock(fmu_cmd_ack_lock_);
    fmu_cmd_ack_received_.store(false, std::memory_order_release);
    fmu_cmd_success_.store(false, std::memory_order_release);
    vehicle_command_pub_->publish(msg);
    if (!fmu_cmd_ack_cv_.wait_for(
        fmu_cmd_ack_lock,
        std::chrono::milliseconds(command_timeout),
        [this] {return fmu_cmd_ack_received_.load(std::memory_order_acquire);}))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "FMU command transmission attempt no. %ld failed",
        attempt + 1);
      continue;
    }
    return true;
  }

  // If control gets here, all transmission attempts failed
  fmu_cmd_ack_received_.store(true, std::memory_order_release);
  return false;
}

/**
 * @brief Notifies a new TakeoffStatus message, if required.
 */
void FlightControlNode::notify_takeoff_status()
{
  {
    std::lock_guard takeoff_status_lock(takeoff_status_lock_);
    bool expected = false;
    if (!takeoff_status_received_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      return;
    }
  }
  takeoff_status_cv_.notify_one();
}

/**
 * @brief Stops the drone at the current position.
 */
void FlightControlNode::stop_drone()
{
  // Get current pose
  state_lock_.lock();
  PoseKit::DynamicPose pose = drone_pose_;
  state_lock_.unlock();

  // Change setpoint to stop the drone at the current position
  change_setpoint(
    Setpoint(
      pose.get_position().x(),
      pose.get_position().y(),
      pose.get_position().z(),
      pose.get_rpy().gamma(),
      Setpoint::Frame::GLOBAL),
    false);
}

/**
 * @brief Checks if the drone is currently stabilized.
 *
 * @param pose Current drone pose.
 *
 * @return Yes or no.
 */
bool FlightControlNode::is_stabilized(const PoseKit::DynamicPose & pose)
{
  double rp_confidence = roll_pitch_stabilization_confidence_;
  double vh_max = v_horz_stabilization_max_;
  double vv_max = v_vert_stabilization_max_;
  if ((abs(pose.get_rpy().alpha()) > rp_confidence) ||
    (abs(pose.get_rpy().beta()) > rp_confidence) ||
    (abs(pose.get_velocity().x()) > vh_max) ||
    (abs(pose.get_velocity().y()) > vh_max) ||
    (abs(pose.get_velocity().z()) > vv_max))
  {
    return false;
  }
  return true;
}

/**
 * @brief Checks if the drone has reached a target position.
 *
 * @param current Current drone pose.
 * @param target Target drone pose.
 * @param confidence_radius Confidence sphere radius.
 *
 * @return Yes or no.
 */
bool FlightControlNode::is_on_target(
  const Eigen::Vector3d & current,
  const Eigen::Vector3d & target,
  double confidence_radius)
{
  if (get_distance(current, target) <= confidence_radius) {
    return true;
  }
  return false;
}

/**
 * @brief Checks if the drone is oriented towards a target heading.
 *
 * @param yaw Current yaw.
 * @param tgt_yaw Target yaw.
 *
 * @return Yes or no.
 */
bool FlightControlNode::is_oriented(double yaw, double tgt_yaw)
{
  double diff = abs(tgt_yaw - yaw);
  if (diff > M_PI) {
    diff = abs((2.0 * M_PI) - diff);
  }
  return diff <= yaw_stabilization_confidence_;
}

/**
 * @brief Returns the euclidean distance between two positions.
 *
 * @param pos1 Position 1.
 * @param pos2 Position 2.
 *
 * @return Distance in meters.
 */
double FlightControlNode::get_distance(
  const Eigen::Vector3d & pos1,
  const Eigen::Vector3d & pos2)
{
  Eigen::Vector3d distance = pos1 - pos2;
  return distance.norm();
}

/**
 * @brief Validates update of agent_node_name parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_agent_node_name(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "FlightControlNode::validate_agent_node_name: Agent node name cannot be empty");
    return false;
  }
  agent_node_name_ = p.as_string();
  return true;
}

/**
 * @brief Validates update of the data_to_px4 parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_data_to_px4(const rclcpp::Parameter & p)
{
  auto new_p = p.as_bool_array();
  if (new_p.size() != 6) {
    RCLCPP_ERROR(
      this->get_logger(),
      "FlightControlNode::validate_data_to_px4: data_to_px4 must be a 6-element bool array");
    return false;
  }
  data_to_px4_ = new_p;
  return true;
}

/**
 * @brief Validates update of the fmu_command_attempts parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_fmu_command_attempts(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    fmu_command_attempts_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_fmu_command_attempts: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the fmu_command_timeout parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_fmu_command_timeout(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    fmu_command_timeout_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_fmu_command_timeout: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the landing_step parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_landing_step(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    landing_step_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_landing_step: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the landing_timeout parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_landing_timeout(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    landing_timeout_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_landing_timeout: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the odometry_topic_name parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_odometry_topic_name(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "FlightControlNode::validate_odometry_topic_name: Odometry topic name cannot be empty");
    return false;
  }
  odometry_topic_name_ = p.as_string();
  return true;
}

/**
 * @brief Validates update of the roll_pitch_stabilization_confidence parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_roll_pitch_stabilization_confidence(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    roll_pitch_stabilization_confidence_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_roll_pitch_stabilization_confidence: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the setpoints_period parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_setpoints_period(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    setpoints_period_ = p.as_int();

    // If timer is up, reset it too
    if (setpoints_timer_ != nullptr) {
      deactivate_setpoints_timer();
      activate_setpoints_timer();
      RCLCPP_INFO(this->get_logger(), "Setpoints timer reset");
    }

    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_setpoints_period: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the takeoff_position_confidence parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_takeoff_position_confidence(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    takeoff_position_confidence_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_takeoff_position_confidence: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the takeoff_timeout parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_takeoff_timeout(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    takeoff_timeout_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_takeoff_timeout: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the travel_sleep_time parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_travel_sleep_time(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    travel_sleep_time_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_travel_sleep_time: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the turn_sleep_time parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_turn_sleep_time(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    turn_sleep_time_ = p.as_int();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_turn_sleep_time: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the turn_step parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_turn_step(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    turn_step_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_turn_step: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the v_horz_stabilization_max parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_v_horz_stabilization_max(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    v_horz_stabilization_max_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_v_horz_stabilization_max: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the v_vert_stabilization_max parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_v_vert_stabilization_max(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    v_vert_stabilization_max_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_v_vert_stabilization_max: Operation in progress");
  return false;
}

/**
 * @brief Validates update of the yaw_stabilization_confidence parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool FlightControlNode::validate_yaw_stabilization_confidence(const rclcpp::Parameter & p)
{
  if (operation_lock_.try_lock()) {
    yaw_stabilization_confidence_ = p.as_double();
    operation_lock_.unlock();
    return true;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "FlightControlNode::validate_yaw_stabilization_confidence: Operation in progress");
  return false;
}

} // namespace FlightControl
