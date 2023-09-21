/**
 * Flight Control flight routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 8, 2022
 */

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Arms the drone.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::arm(const ArmGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Arm::Result>();

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    // Try to arm the drone
    if (arm_drone(result->result)) {
      operation_lock_.unlock();
      goal_handle->succeed(result);
      RCLCPP_WARN(this->get_logger(), "Drone ARMED");
    } else {
      operation_lock_.unlock();
      goal_handle->abort(result);
    }
  }
}

/**
 * @brief Disarms the drone.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::disarm(const DisarmGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Disarm::Result>();

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    // Try to send the command, waiting for both the ACK and the TakeoffStatus update
    int64_t command_timeout = fmu_command_timeout_;
    takeoff_status_received_.store(false, std::memory_order_release);
    if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)) {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "DISARM command transmission failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Command transmission failed");
      goal_handle->abort(result);
      return;
    }
    if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "DISARM command failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("FMU command failed");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "DISARM command sent");
    {
      std::unique_lock takeoff_status_lock(takeoff_status_lock_);
      if (!takeoff_status_cv_.wait_for(
          takeoff_status_lock,
          std::chrono::milliseconds(command_timeout),
          [this] {return takeoff_status_received_.load(std::memory_order_acquire);}))
      {
        takeoff_status_received_.store(true, std::memory_order_release);
        armed_.store(false, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(), "FMU disarming state update not received");
      }
    }
    operation_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    result->result.set__error_msg("");
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Drone DISARMED");
  }
}

/**
 * @brief Performs the landing procedure.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::landing(const LandingGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Landing::Result>();
  auto feedback = std::make_shared<Landing::Feedback>();

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    // Get current position setpoint (abort landing if position control is not engaged)
    setpoint_lock_.lock();
    double curr_x = fmu_setpoint_.x;
    double curr_y = fmu_setpoint_.y;
    double yaw_angle = fmu_setpoint_.yaw;
    ControlModes control_mode = fmu_setpoint_.control_mode;
    Setpoint land_setp = fmu_setpoint_;
    setpoint_lock_.unlock();
    if (control_mode != ControlModes::POSITION) {
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "Position control not engaged, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Position control not engaged");
      goal_handle->abort(result);
      return;
    }

    // Get minimums altitude
    double min_altitude = goal_handle->get_goal()->minimums.point.z;

    // Descend to minimum altitude
    RCLCPP_INFO(this->get_logger(), "Descent initiated");
    int64_t landing_timeout = landing_timeout_;
    int64_t travel_sleep_time = travel_sleep_time_;
    double landing_step = landing_step_;
    rclcpp::Time landing_start = clock_.now();
    bool ok = false;
    de_ascending_.store(true, std::memory_order_release);
    while (true) {
      // Check if cancellation was requested
      if (goal_handle->is_canceling()) {
        stop_drone();
        operation_lock_.unlock();
        de_ascending_.store(false, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(), "Landing canceled");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::SUCCESS);
        goal_handle->canceled(result);
        return;
      }

      // Check if the minimum altitude has been reached
      state_lock_.lock();
      PoseKit::DynamicPose curr_pose = drone_pose_;
      state_lock_.unlock();
      double curr_z = curr_pose.get_position().z();
      if (curr_z <= min_altitude) {
        break;
      }

      // Lower the setpoint (should never fail now)
      if (abs(land_setp.z - curr_z) < landing_step) {
        land_setp.z -= landing_step;
        ok = change_setpoint(land_setp);
      } else {
        ok = true;
      }

      // Keep in mind that shit happens
      rclcpp::Time current_time = clock_.now();
      ok = ok && (current_time - landing_start) < rclcpp::Duration(landing_timeout / 1000, 0);
      if (!ok) {
        operation_lock_.unlock();
        de_ascending_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(), "Landing decision altitude not reached");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::ERROR);
        result->result.set__error_msg("Landing decision altitude not reached");
        goal_handle->abort(result);
        return;
      }

      // Publish current pose
      feedback->set__pose(curr_pose.to_pose_stamped());
      goal_handle->publish_feedback(feedback);

      // Relinquish the CPU while the drone does its thing
      std::this_thread::sleep_for(std::chrono::milliseconds(travel_sleep_time));
    }
    de_ascending_.store(false, std::memory_order_release);
    RCLCPP_INFO(this->get_logger(), "Minimums, landing");

    // Get current map -> odom transform
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
          "FlightControlNode::landing: TF exception: %s",
          e.what());
        operation_lock_.unlock();
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::ERROR);
        result->result.set__error_msg("TF subsystem failure");
        goal_handle->abort(result);
        return;
      }
    }

    // Try to send the LAND MODE transition command
    Eigen::AngleAxisd land_yaw_map(yaw_angle, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_odom_to_map = tf2::transformToEigen(map_to_odom).inverse().rotation();
    double land_yaw_odom = Eigen::EulerAnglesXYZd(R_odom_to_map * land_yaw_map).gamma();
    takeoff_status_received_.store(false, std::memory_order_release);
    if (!send_fmu_command(
        VehicleCommand::VEHICLE_CMD_NAV_LAND,
        NAN, NAN, NAN,
        -land_yaw_odom,
        NAN, NAN,
        0.0))
    {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "LAND command transmission failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::ERROR);
      result->result.set__error_msg("Command transmission failed");
      goal_handle->abort(result);
      return;
    }
    if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "LAND command failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::ERROR);
      result->result.set__error_msg("FMU command failed");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "LAND MODE engaged");

    // Disable setpoints stream
    deactivate_setpoints_timer();

    RCLCPP_WARN(
      this->get_logger(),
      "Landing at (%f, %f), with heading: %f°",
      curr_x, curr_y,
      yaw_angle * 180.0 / M_PI);

    // Wait for TakeoffStatus update
    {
      std::unique_lock takeoff_status_lock(takeoff_status_lock_);
      if (!takeoff_status_cv_.wait_for(
          takeoff_status_lock,
          std::chrono::milliseconds(landing_timeout),
          [this] {return takeoff_status_received_.load(std::memory_order_acquire);}))
      {
        takeoff_status_received_.store(true, std::memory_order_release);
        operation_lock_.unlock();
        RCLCPP_ERROR(this->get_logger(), "Landing failed, FMU state update not received");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::ERROR);
        result->result.set__error_msg("FMU state update not received");
        goal_handle->abort(result);
        return;
      }
    }

    operation_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    result->result.set__error_msg("");
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Drone LANDED");
  }
}

/**
 * @brief Moves the drone to a target position.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::reach(const ReachGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Reach::Result>();
  auto feedback = std::make_shared<Reach::Feedback>();

  PoseKit::DynamicPose target_pose(goal_handle->get_goal()->target_pose);
  double confidence_radius = abs(goal_handle->get_goal()->reach_radius);
  bool stabilize = goal_handle->get_goal()->stop_at_target;

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    // Try to change the current setpoint
    if (!change_setpoint(
        Setpoint(
          target_pose.get_position().x(),
          target_pose.get_position().y(),
          target_pose.get_position().z(),
          target_pose.get_rpy().gamma())))
    {
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "Invalid target setpoint, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Invalid target setpoint");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Going at (%f, %f, %f), with heading: %f°",
      target_pose.get_position().x(), target_pose.get_position().y(),
      target_pose.get_position().z(),
      target_pose.get_rpy().gamma() * 180.0 / M_PI);

    // Wait for the drone to safely reach the target position
    int64_t travel_sleep_time = travel_sleep_time_;
    while (true) {
      // Relinquish the CPU while the drone does its thing
      std::this_thread::sleep_for(std::chrono::milliseconds(travel_sleep_time));

      // Check the current position
      state_lock_.lock();
      PoseKit::DynamicPose current_pose = drone_pose_;
      state_lock_.unlock();
      bool is_done =
        (!stabilize || is_stabilized(current_pose)) &&
        is_on_target(current_pose.get_position(), target_pose.get_position(), confidence_radius) &&
        is_oriented(current_pose.get_rpy().gamma(), target_pose.get_rpy().gamma());
      if (is_done) {
        break;
      }

      // Check if the drone must be stopped
      if (goal_handle->is_canceling()) {
        stop_drone();
        operation_lock_.unlock();
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::SUCCESS);
        result->result.set__error_msg("Operation canceled");
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Full stop requested");
        return;
      }

      // Send some feedback
      feedback->set__current_pose(current_pose.to_pose_stamped());
      feedback->set__distance_from_target(
        get_distance(
          current_pose.get_position(),
          target_pose.get_position()));
      goal_handle->publish_feedback(feedback);
    }

    operation_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    result->result.set__error_msg("");
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Target position reached");
  }
}

/**
 * @brief Performs the takeoff procedure.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::takeoff(const TakeoffGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Takeoff::Result>();
  auto feedback = std::make_shared<Takeoff::Feedback>();

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    // Takeoff coordinates are current ones (rounded to cm)
    state_lock_.lock();
    double takeoff_x = drone_pose_.get_position().x();
    double takeoff_y = drone_pose_.get_position().y();
    double takeoff_yaw = drone_pose_.get_rpy().gamma();
    state_lock_.unlock();
    takeoff_x *= 100.0;
    takeoff_y *= 100.0;
    takeoff_x = floor(takeoff_x);
    takeoff_y = floor(takeoff_y);
    takeoff_x /= 100.0;
    takeoff_y /= 100.0;

    // Takeoff altitude is given
    double takeoff_z = abs(goal_handle->get_goal()->takeoff_pose.pose.position.z);

    // Set takeoff setpoint and activate setpoints stream
    if (!change_setpoint(
        Setpoint(
          takeoff_x,
          takeoff_y,
          takeoff_z,
          takeoff_yaw)))
    {
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "Invalid takeoff setpoint, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Invalid takeoff setpoint");
      goal_handle->abort(result);
      return;
    }
    if (setpoints_timer_ == nullptr) {
      activate_setpoints_timer();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Let PX4 adapt
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Attempting takeoff at (%f, %f), altitude: %f m, heading: %f°",
      takeoff_x, takeoff_y, takeoff_z,
      takeoff_yaw * 180.0 / M_PI);

    // Arm the drone, if necessary
    if (!armed_.load(std::memory_order_acquire)) {
      if (arm_drone(result->result)) {
        RCLCPP_WARN(this->get_logger(), "Drone ARMED");
      } else {
        operation_lock_.unlock();
        goal_handle->abort(result);
        return;
      }
    }

    // Try to send the OFFBOARD MODE transition command
    takeoff_status_received_.store(false, std::memory_order_release);
    if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)) {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "OFFBOARD command transmission failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::ERROR);
      result->result.set__error_msg("Command transmission failed");
      goal_handle->abort(result);
      return;
    }
    if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
      takeoff_status_received_.store(true, std::memory_order_release);
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "OFFBOARD command failed, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::ERROR);
      result->result.set__error_msg("FMU command failed");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "OFFBOARD MODE engaged");

    // Wait for TakeoffStatus update
    int64_t takeoff_timeout = takeoff_timeout_;
    {
      std::unique_lock takeoff_status_lock(takeoff_status_lock_);
      if (!takeoff_status_cv_.wait_for(
          takeoff_status_lock,
          std::chrono::milliseconds(takeoff_timeout),
          [this] {return takeoff_status_received_.load(std::memory_order_acquire);}))
      {
        takeoff_status_received_.store(true, std::memory_order_release);
        operation_lock_.unlock();
        RCLCPP_ERROR(this->get_logger(), "Takeoff failed, FMU state update not received");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::ERROR);
        result->result.set__error_msg("FMU state update not received");
        goal_handle->abort(result);
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Takeoff detected");

    // Wait for the drone to safely reach the takeoff altitude
    int64_t travel_sleep_time = travel_sleep_time_;
    double takeoff_position_confidence = takeoff_position_confidence_;
    PoseKit::Pose takeoff_pose(takeoff_x, takeoff_y, takeoff_z, takeoff_yaw, Header{});
    rclcpp::Time takeoff_start = clock_.now();
    de_ascending_.store(true, std::memory_order_release);
    while (true) {
      // Relinquish the CPU while the drone does its thing
      std::this_thread::sleep_for(std::chrono::milliseconds(travel_sleep_time));

      // Check if cancellation was requested
      if (goal_handle->is_canceling()) {
        stop_drone();
        operation_lock_.unlock();
        de_ascending_.store(false, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(), "Takeoff canceled");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::SUCCESS);
        goal_handle->canceled(result);
        return;
      }

      // Check the current position
      state_lock_.lock();
      PoseKit::DynamicPose current_pose = drone_pose_;
      state_lock_.unlock();
      bool is_done =
        is_stabilized(current_pose) &&
        is_on_target(
        current_pose.get_position(),
        takeoff_pose.get_position(),
        takeoff_position_confidence) &&
        is_oriented(current_pose.get_rpy().gamma(), takeoff_pose.get_rpy().gamma());
      if (is_done) {
        break;
      }

      // Send some feedback to who's waiting
      feedback->set__current_pose(current_pose.to_pose_stamped());
      goal_handle->publish_feedback(feedback);

      // Keep in mind that shit happens
      rclcpp::Time current_time = clock_.now();
      if ((current_time - takeoff_start) >= rclcpp::Duration(takeoff_timeout / 1000, 0)) {
        operation_lock_.unlock();
        de_ascending_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(), "Takeoff altitude not reached");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::ERROR);
        result->result.set__error_msg("Takeoff altitude not reached");
        goal_handle->abort(result);
        return;
      }
    }

    operation_lock_.unlock();
    de_ascending_.store(false, std::memory_order_release);
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    result->result.set__error_msg("");
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Drone AIRBORNE");
  }
}

/**
 * @brief Performs a turn up to the given heading.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::turn(const TurnGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Turn::Result>();

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  {
    std::unique_lock<std::mutex> stream_reset_lk(stream_reset_lock_);

    double target_yaw = goal_handle->get_goal()->heading;

    // Position setpoint is the current position
    state_lock_.lock();
    Eigen::Vector3d position_setpoint(
      drone_pose_.get_position().x(),
      drone_pose_.get_position().y(),
      drone_pose_.get_position().z());
    double current_yaw = drone_pose_.get_rpy().gamma();
    state_lock_.unlock();
    Setpoint turn_setpoint(
      position_setpoint(0),
      position_setpoint(1),
      position_setpoint(2),
      0.0);

    RCLCPP_WARN(
      this->get_logger(),
      "Turning from heading %f° to %f° at (%f, %f, %f)",
      current_yaw * 180.0 / M_PI,
      target_yaw * 180.0 / M_PI,
      position_setpoint(0),
      position_setpoint(1),
      position_setpoint(2));

    // Do the turn
    rclcpp_action::ResultCode turn_res;
    double start_yaw = current_yaw;
    if (abs(current_yaw - target_yaw) > M_PI) {
      // Must get to PI first
      turn_res = do_turn(
        goal_handle,
        turn_setpoint,
        current_yaw,
        M_PI * (current_yaw > 0.0 ? 1.0 : -1.0));

      if (turn_res == rclcpp_action::ResultCode::ABORTED) {
        // Something went wrong
        operation_lock_.unlock();
        RCLCPP_ERROR(this->get_logger(), "Invalid yaw setpoint, aborting");
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::FAILED);
        result->result.set__error_msg("Invalid yaw setpoint");
        goal_handle->abort(result);
        return;
      }

      if (turn_res == rclcpp_action::ResultCode::CANCELED) {
        // The action was canceled during the turn
        operation_lock_.unlock();
        result->result.header.set__stamp(this->get_clock()->now());
        result->result.header.set__frame_id(link_namespace_ + "fmu_link");
        result->result.set__result(CommandResultStamped::FAILED);
        result->result.set__error_msg("Full stop requested");
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Full stop requested");
        return;
      }

      start_yaw = M_PI * (current_yaw > 0.0 ? -1.0 : 1.0);
    }

    // Do the final part of the turn
    turn_res = do_turn(
      goal_handle,
      turn_setpoint,
      start_yaw,
      target_yaw);

    if (turn_res == rclcpp_action::ResultCode::ABORTED) {
      // Something went wrong
      operation_lock_.unlock();
      RCLCPP_ERROR(this->get_logger(), "Invalid yaw setpoint, aborting");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Invalid yaw setpoint");
      goal_handle->abort(result);
      return;
    }

    if (turn_res == rclcpp_action::ResultCode::CANCELED) {
      // The action was canceled during the turn
      operation_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(link_namespace_ + "fmu_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Full stop requested");
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Full stop requested");
      return;
    }

    operation_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(link_namespace_ + "fmu_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    result->result.set__error_msg("");
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Target heading reached");
  }
}

/**
 * @brief Wraps the arming procedure, necessary in multiple sections.
 *
 * @param result CommandResult message to populate.
 */
bool FlightControlNode::arm_drone(CommandResultStamped & result)
{
  // Check if PX4 is sending pose feedback
  rclcpp::Time curr_time = clock_.now();
  rclcpp::Time last_pose_timestamp;
  state_lock_.lock();
  last_pose_timestamp = last_pose_timestamp_;
  state_lock_.unlock();
  if ((curr_time - last_pose_timestamp) > rclcpp::Duration(0, 100000000)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arming denied, no pose feedback from PX4");
    result.header.set__stamp(this->get_clock()->now());
    result.header.set__frame_id(link_namespace_ + "fmu_link");
    result.set__result(CommandResultStamped::ERROR);
    result.set__error_msg("No pose feedback from PX4");
    return false;
  }

  // Try to send the command, waiting for both the ACK and the TakeoffStatus update
  int64_t command_timeout = fmu_command_timeout_;
  takeoff_status_received_.store(false, std::memory_order_release);
  if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)) {
    RCLCPP_ERROR(this->get_logger(), "ARM command transmission failed, aborting");
    result.header.set__stamp(this->get_clock()->now());
    result.header.set__frame_id(link_namespace_ + "fmu_link");
    result.set__result(CommandResultStamped::ERROR);
    result.set__error_msg("Command transmission failed");
    takeoff_status_received_.store(true, std::memory_order_release);
    return false;
  }
  if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(this->get_logger(), "ARM command failed, aborting");
    result.header.set__stamp(this->get_clock()->now());
    result.header.set__frame_id(link_namespace_ + "fmu_link");
    result.set__result(CommandResultStamped::ERROR);
    result.set__error_msg("FMU command failed");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "ARM command sent");
  {
    std::unique_lock takeoff_status_lock(takeoff_status_lock_);
    if (!takeoff_status_cv_.wait_for(
        takeoff_status_lock,
        std::chrono::milliseconds(command_timeout),
        [this] {return takeoff_status_received_.load(std::memory_order_acquire);}))
    {
      takeoff_status_received_.store(true, std::memory_order_release);
      armed_.store(true, std::memory_order_release);
      RCLCPP_WARN(this->get_logger(), "FMU arming state update not received");
    }
  }
  result.header.set__stamp(this->get_clock()->now());
  result.header.set__frame_id(link_namespace_ + "fmu_link");
  result.set__result(CommandResultStamped::SUCCESS);
  result.set__error_msg("");
  return true;
}

/**
 * @brief Incrementally performs a turn.
 *
 * @param goal_handle Turn action goal handle pointer.
 * @param turn_setpoint Setpoint data to use to move the drone.
 * @param start_yaw Initial yaw angle.
 * @param target_yaw Final yaw angle.
 *
 * @return Action terminal state to set.
 */
rclcpp_action::ResultCode FlightControlNode::do_turn(
  const TurnGoalHandleSharedPtr goal_handle,
  Setpoint & turn_setpoint,
  double start_yaw,
  double target_yaw)
{
  // Consistency check (should never happen)
  if (start_yaw == target_yaw) {
    return rclcpp_action::ResultCode::SUCCEEDED;
  }

  auto feedback = std::make_shared<Turn::Feedback>();

  // Left or right?
  double direction = (target_yaw - start_yaw) > 0.0 ? 1.0 : -1.0;

  // Turn incrementally, waiting for the drone to reach each step
  int64_t turn_sleep_time = turn_sleep_time_;
  double turn_step = turn_step_;
  double increment = direction * turn_step;
  double yaw_step, initial_yaw = start_yaw;
  do {
    // Change the position setpoint with a new turn step
    yaw_step =
      direction * (initial_yaw + increment) <=
      direction * target_yaw ? (initial_yaw + increment) : target_yaw;
    turn_setpoint.yaw = yaw_step;
    if (!change_setpoint(turn_setpoint)) {
      return rclcpp_action::ResultCode::ABORTED;
    }

    while (true) {
      // Relinquish the CPU while the drone does its thing
      std::this_thread::sleep_for(std::chrono::milliseconds(turn_sleep_time));

      // Check the current heading
      state_lock_.lock();
      PoseKit::DynamicPose current_pose = drone_pose_;
      state_lock_.unlock();
      if (is_oriented(drone_pose_.get_rpy().gamma(), yaw_step)) {
        break;
      }

      // Check if the drone must be stopped
      if (goal_handle->is_canceling()) {
        stop_drone();
        return rclcpp_action::ResultCode::CANCELED;
      }

      // Send some feedback
      feedback->set__current_yaw(current_pose.get_rpy().gamma());
      feedback->set__current_yaw_deg(current_pose.get_rpy().gamma() * 180.0 / M_PI);
      goal_handle->publish_feedback(feedback);
    }

    // Update initial heading
    initial_yaw = yaw_step;
  } while (yaw_step != target_yaw);

  return rclcpp_action::ResultCode::SUCCEEDED;
}

} // namespace FlightControl
