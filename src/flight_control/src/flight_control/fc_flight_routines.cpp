/**
 * Flight Control flight routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 8, 2022
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
 * @brief Arms the drone.
 *
 * @param goal_handle Action goal handle pointer.
 */
void FlightControlNode::arm(const ArmGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Arm::Result>();

  // Check if some other operation is in progress
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Try to arm the drone
  if (arm_drone(result->result)) {
    pthread_spin_unlock(&(this->operation_lock_));
    goal_handle->succeed(result);
    RCLCPP_WARN(this->get_logger(), "Drone ARMED");
  } else {
    pthread_spin_unlock(&(this->operation_lock_));
    goal_handle->abort(result);
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
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Try to send the command, waiting for both the ACK and the TakeoffStatus update
  int64_t command_timeout = fmu_command_timeout_;
  takeoff_status_received_.store(false, std::memory_order_release);
  if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)) {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "DISARM command transmission failed, aborting");
    result->result.set__result(CommandResult::ERROR);
    result->result.set__error_msg("Command transmission failed");
    goal_handle->abort(result);
    return;
  }
  if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "DISARM command failed, aborting");
    result->result.set__result(CommandResult::ERROR);
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
  pthread_spin_unlock(&(this->operation_lock_));
  result->result.set__result(CommandResult::SUCCESS);
  result->result.set__error_msg("");
  goal_handle->succeed(result);
  RCLCPP_WARN(this->get_logger(), "Drone DISARMED");
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
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Get current position setpoint (abort landing if position control is not engaged)
  pthread_spin_lock(&(this->setpoint_lock_));
  float curr_x = fmu_setpoint_.x;
  float curr_y = fmu_setpoint_.y;
  float yaw_angle = fmu_setpoint_.yaw;
  ControlModes control_mode = fmu_setpoint_.control_mode;
  pthread_spin_unlock(&(this->setpoint_lock_));
  if (control_mode != ControlModes::POSITION) {
    RCLCPP_ERROR(this->get_logger(), "Position control not engaged, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Position control not engaged");
    goal_handle->abort(result);
    return;
  }

  // Try to send the LAND MODE transition command
  takeoff_status_received_.store(false, std::memory_order_release);
  if (!send_fmu_command(
      VehicleCommand::VEHICLE_CMD_NAV_LAND,
      NAN, NAN, NAN,
      yaw_angle,
      NAN, NAN,
      0.0))
  {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "LAND command transmission failed, aborting");
    result->result.set__result(CommandResult::ERROR);
    result->result.set__error_msg("Command transmission failed");
    goal_handle->abort(result);
    return;
  }
  if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "LAND command failed, aborting");
    result->result.set__result(CommandResult::ERROR);
    result->result.set__error_msg("FMU command failed");
    goal_handle->abort(result);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "LAND MODE engaged");
  feedback->set__fmu_state(Landing::Feedback::LANDING);
  goal_handle->publish_feedback(feedback);

  // Disable setpoints stream
  deactivate_setpoints_timer();

  RCLCPP_WARN(
    this->get_logger(),
    "Landing at (%f, %f), with heading: %f°",
    curr_x, curr_y,
    yaw_angle * 180.0f / M_PIf32);

  // Wait for TakeoffStatus update
  int64_t landing_timeout = landing_timeout_;
  {
    std::unique_lock takeoff_status_lock(takeoff_status_lock_);
    if (!takeoff_status_cv_.wait_for(
        takeoff_status_lock,
        std::chrono::milliseconds(landing_timeout),
        [this] {return takeoff_status_received_.load(std::memory_order_acquire);}))
    {
      takeoff_status_received_.store(true, std::memory_order_release);
      pthread_spin_unlock(&(this->operation_lock_));
      RCLCPP_ERROR(this->get_logger(), "Landing failed, FMU state update not received");
      result->result.set__result(CommandResult::ERROR);
      result->result.set__error_msg("FMU state update not received");
      goal_handle->abort(result);
      return;
    }
  }

  pthread_spin_unlock(&(this->operation_lock_));
  result->result.set__result(CommandResult::SUCCESS);
  result->result.set__error_msg("");
  goal_handle->succeed(result);
  RCLCPP_WARN(this->get_logger(), "Drone LANDED");
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

  DronePose target_pose(
    goal_handle->get_goal()->position,
    goal_handle->get_goal()->heading);
  float confidence_radius = abs(goal_handle->get_goal()->reach_radius);
  bool stabilize = goal_handle->get_goal()->stabilize;

  // Try to change the current setpoint
  if (!change_setpoint(
      Setpoint(
        target_pose.position(0),
        target_pose.position(1),
        target_pose.position(2),
        target_pose.rpy.gamma())))
  {
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "Invalid target setpoint, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Invalid target setpoint");
    goal_handle->abort(result);
    return;
  }
  RCLCPP_WARN(
    this->get_logger(),
    "Going at (%f, %f, %f), with heading: %f°",
    target_pose.position(0), target_pose.position(1), target_pose.position(2),
    target_pose.rpy.gamma() * 180.0f / M_PIf32);

  // Wait for the drone to safely reach the target position
  int64_t travel_sleep_time = travel_sleep_time_;
  while (true) {
    // Relinquish the CPU while the drone does its thing
    std::this_thread::sleep_for(std::chrono::milliseconds(travel_sleep_time));

    // Check the current position
    pthread_spin_lock(&(this->state_lock_));
    DronePose current_pose = drone_pose_;
    pthread_spin_unlock(&(this->state_lock_));
    bool is_done =
      (!stabilize || is_stabilized(current_pose)) &&
      is_on_target(current_pose.position, target_pose.position, confidence_radius) &&
      is_oriented(current_pose.rpy.gamma(), target_pose.rpy.gamma());
    if (is_done) {
      break;
    }

    // Check if the drone must be stopped
    if (goal_handle->is_canceling()) {
      stop_drone(current_pose);
      pthread_spin_unlock(&(this->operation_lock_));
      result->result.set__result(CommandResult::FAILED);
      result->result.set__error_msg("Operation canceled");
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Full stop requested");
      return;
    }

    // Send some feedback
    feedback->set__current_x(current_pose.position(0));
    feedback->set__current_y(current_pose.position(1));
    feedback->set__current_z(current_pose.position(2));
    feedback->set__current_yaw(current_pose.rpy.gamma());
    feedback->set__current_distance(get_distance(current_pose.position, target_pose.position));
    goal_handle->publish_feedback(feedback);
  }

  pthread_spin_unlock(&(this->operation_lock_));
  result->result.set__result(CommandResult::SUCCESS);
  result->result.set__error_msg("");
  goal_handle->succeed(result);
  RCLCPP_WARN(this->get_logger(), "Target position reached");
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
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Takeoff coordinates are current ones (rounded to cm)
  pthread_spin_lock(&(this->state_lock_));
  float takeoff_x = drone_pose_.position(0);
  float takeoff_y = drone_pose_.position(1);
  float takeoff_yaw = drone_pose_.rpy.gamma();
  pthread_spin_unlock(&(this->state_lock_));
  takeoff_x *= 100.0f;
  takeoff_y *= 100.0f;
  takeoff_x = floor(takeoff_x);
  takeoff_y = floor(takeoff_y);
  takeoff_x /= 100.0f;
  takeoff_y /= 100.0f;

  // Takeoff altitude is given
  float takeoff_z = -abs(goal_handle->get_goal()->takeoff_altitude);

  // Set takeoff setpoint and activate setpoints stream
  if (!change_setpoint(
      Setpoint(
        takeoff_x,
        takeoff_y,
        takeoff_z,
        takeoff_yaw)))
  {
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "Invalid takeoff setpoint, aborting");
    result->result.set__result(CommandResult::FAILED);
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
    takeoff_yaw * 180.0f / M_PIf32);

  // Arm the drone, if necessary
  if (!armed_.load(std::memory_order_acquire)) {
    if (arm_drone(result->result)) {
      RCLCPP_WARN(this->get_logger(), "Drone ARMED");
    } else {
      pthread_spin_unlock(&(this->operation_lock_));
      goal_handle->abort(result);
      return;
    }
  }

  // Try to send the OFFBOARD MODE transition command
  takeoff_status_received_.store(false, std::memory_order_release);
  if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)) {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "OFFBOARD command transmission failed, aborting");
    result->result.set__result(CommandResult::ERROR);
    result->result.set__error_msg("Command transmission failed");
    goal_handle->abort(result);
    return;
  }
  if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
    takeoff_status_received_.store(true, std::memory_order_release);
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "OFFBOARD command failed, aborting");
    result->result.set__result(CommandResult::ERROR);
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
      pthread_spin_unlock(&(this->operation_lock_));
      RCLCPP_ERROR(this->get_logger(), "Takeoff failed, FMU state update not received");
      result->result.set__result(CommandResult::ERROR);
      result->result.set__error_msg("FMU state update not received");
      goal_handle->abort(result);
      return;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff detected");

  // Wait for the drone to safely reach the takeoff altitude
  int64_t travel_sleep_time = travel_sleep_time_;
  float takeoff_position_confidence = takeoff_position_confidence_;
  DronePose takeoff_pose(takeoff_x, takeoff_y, takeoff_z, takeoff_yaw);
  rclcpp::Time takeoff_start = clock_.now();
  while (true) {
    // Relinquish the CPU while the drone does its thing
    std::this_thread::sleep_for(std::chrono::milliseconds(travel_sleep_time));

    // Check the current position
    pthread_spin_lock(&(this->state_lock_));
    DronePose current_pose = drone_pose_;
    pthread_spin_unlock(&(this->state_lock_));
    bool is_done =
      is_stabilized(current_pose) &&
      is_on_target(current_pose.position, takeoff_pose.position, takeoff_position_confidence) &&
      is_oriented(current_pose.rpy.gamma(), takeoff_pose.rpy.gamma());
    if (is_done) {
      break;
    }

    // Send some feedback to who's waiting
    feedback->set__altitude(current_pose.position(2));
    goal_handle->publish_feedback(feedback);

    // Keep in mind that shit happens
    rclcpp::Time current_time = clock_.now();
    if ((current_time - takeoff_start) >= rclcpp::Duration(takeoff_timeout / 1000, 0)) {
      pthread_spin_unlock(&(this->operation_lock_));
      RCLCPP_ERROR(this->get_logger(), "Takeoff altitude not reached");
      result->result.set__result(CommandResult::ERROR);
      result->result.set__error_msg("Takeoff altitude not reached");
      goal_handle->abort(result);
      return;
    }
  }

  pthread_spin_unlock(&(this->operation_lock_));
  result->result.set__result(CommandResult::SUCCESS);
  result->result.set__error_msg("");
  goal_handle->succeed(result);
  RCLCPP_WARN(this->get_logger(), "Drone AIRBORNE");
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
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  float target_yaw = goal_handle->get_goal()->yaw;

  // Position setpoint is the last saved one
  pthread_spin_lock(&(this->setpoint_lock_));
  Eigen::Vector3f position_setpoint(
    fmu_setpoint_.x,
    fmu_setpoint_.y,
    fmu_setpoint_.z);
  float current_yaw = fmu_setpoint_.yaw;
  pthread_spin_unlock(&(this->setpoint_lock_));
  Setpoint turn_setpoint(
    position_setpoint(0),
    position_setpoint(1),
    position_setpoint(2),
    0.0f);

  RCLCPP_WARN(
    this->get_logger(),
    "Turning from heading %f° to %f° at (%f, %f, %f)",
    current_yaw * 180.0f / M_PIf32,
    target_yaw * 180.0f / M_PIf32,
    position_setpoint(0),
    position_setpoint(1),
    position_setpoint(2));

  // Do the turn
  rclcpp_action::ResultCode turn_res;
  float start_yaw = current_yaw;
  if (abs(current_yaw - target_yaw) > M_PIf32) {
    // Must get to PI first
    turn_res = do_turn(
      goal_handle,
      turn_setpoint,
      current_yaw,
      M_PIf32 * (current_yaw > 0.0 ? 1.0 : -1.0));

    if (turn_res == rclcpp_action::ResultCode::ABORTED) {
      // Something went wrong
      pthread_spin_unlock(&(this->operation_lock_));
      RCLCPP_ERROR(this->get_logger(), "Invalid yaw setpoint, aborting");
      result->result.set__result(CommandResult::FAILED);
      result->result.set__error_msg("Invalid yaw setpoint");
      goal_handle->abort(result);
      return;
    }

    if (turn_res == rclcpp_action::ResultCode::CANCELED) {
      // The action was canceled during the turn
      pthread_spin_unlock(&(this->operation_lock_));
      result->result.set__result(CommandResult::FAILED);
      result->result.set__error_msg("Full stop requested");
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Full stop requested");
      return;
    }

    start_yaw = M_PIf32 * (current_yaw > 0.0 ? -1.0 : 1.0);
  }

  // Do the final part of the turn
  turn_res = do_turn(
    goal_handle,
    turn_setpoint,
    start_yaw,
    target_yaw);

  if (turn_res == rclcpp_action::ResultCode::ABORTED) {
    // Something went wrong
    pthread_spin_unlock(&(this->operation_lock_));
    RCLCPP_ERROR(this->get_logger(), "Invalid yaw setpoint, aborting");
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Invalid yaw setpoint");
    goal_handle->abort(result);
    return;
  }

  if (turn_res == rclcpp_action::ResultCode::CANCELED) {
    // The action was canceled during the turn
    pthread_spin_unlock(&(this->operation_lock_));
    result->result.set__result(CommandResult::FAILED);
    result->result.set__error_msg("Full stop requested");
    goal_handle->canceled(result);
    RCLCPP_WARN(this->get_logger(), "Full stop requested");
    return;
  }

  pthread_spin_unlock(&(this->operation_lock_));
  result->result.set__result(CommandResult::SUCCESS);
  result->result.set__error_msg("");
  goal_handle->succeed(result);
  RCLCPP_WARN(this->get_logger(), "Target heading reached");
}

/**
 * @brief Wraps the arming procedure, necessary in multiple sections.
 *
 * @param result CommandResult message to populate.
 */
bool FlightControlNode::arm_drone(CommandResult & result)
{
  // Check if PX4 is sending pose feedback
  rclcpp::Time curr_time = clock_.now();
  rclcpp::Time last_pose_timestamp;
  pthread_spin_lock(&(this->state_lock_));
  last_pose_timestamp = last_pose_timestamp_;
  pthread_spin_unlock(&(this->state_lock_));
  if ((curr_time - last_pose_timestamp) > rclcpp::Duration(0, 100000000)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arming denied, no pose feedback from PX4");
    result.set__result(CommandResult::ERROR);
    result.set__error_msg("No pose feedback from PX4");
    return false;
  }

  // Try to send the command, waiting for both the ACK and the TakeoffStatus update
  int64_t command_timeout = fmu_command_timeout_;
  takeoff_status_received_.store(false, std::memory_order_release);
  if (!send_fmu_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)) {
    RCLCPP_ERROR(this->get_logger(), "ARM command transmission failed, aborting");
    result.set__result(CommandResult::ERROR);
    result.set__error_msg("Command transmission failed");
    takeoff_status_received_.store(true, std::memory_order_release);
    return false;
  }
  if (!fmu_cmd_success_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(this->get_logger(), "ARM command failed, aborting");
    result.set__result(CommandResult::ERROR);
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
  result.set__result(CommandResult::SUCCESS);
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
  float start_yaw,
  float target_yaw)
{
  // Consistency check (should never happen)
  if (start_yaw == target_yaw) {
    return rclcpp_action::ResultCode::SUCCEEDED;
  }

  auto feedback = std::make_shared<Turn::Feedback>();

  // Left or right?
  float direction = (target_yaw - start_yaw) > 0.0f ? 1.0f : -1.0f;

  // Turn incrementally, waiting for the drone to reach each step
  int64_t turn_sleep_time = turn_sleep_time_;
  float turn_step = turn_step_;
  float increment = direction * turn_step;
  float yaw_step, initial_yaw = start_yaw;
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
      pthread_spin_lock(&(this->state_lock_));
      DronePose current_pose = drone_pose_;
      pthread_spin_unlock(&(this->state_lock_));
      if (is_oriented(drone_pose_.rpy.gamma(), yaw_step)) {
        break;
      }

      // Check if the drone must be stopped
      if (goal_handle->is_canceling()) {
        stop_drone(current_pose);
        return rclcpp_action::ResultCode::CANCELED;
      }

      // Send some feedback
      feedback->set__current_yaw(current_pose.rpy.gamma());
      feedback->set__current_yaw_deg(current_pose.rpy.gamma() * 180.0f / M_PIf32);
      goal_handle->publish_feedback(feedback);
    }

    // Update initial heading
    initial_yaw = yaw_step;
  } while (yaw_step != target_yaw);

  return rclcpp_action::ResultCode::SUCCEEDED;
}

} // namespace FlightControl
