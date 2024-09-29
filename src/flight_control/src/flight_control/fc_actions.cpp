/**
 * Flight Control action callbacks.
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

#define UNUSED(arg) (void)(arg)

#include <flight_control/flight_control.hpp>

namespace flight_stack
{

/**
 * @brief Handles a new Arm goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_arm_goal(
  const rclcpp_action::GoalUUID & uuid,
  ArmGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);
  RCLCPP_INFO(this->get_logger(), "Received arming request");
  if (airborne_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arming request rejected, drone is airborne");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Disarm goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_disarm_goal(
  const rclcpp_action::GoalUUID & uuid,
  DisarmGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);
  RCLCPP_INFO(this->get_logger(), "Received disarming request");
  if (airborne_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Disarming request rejected, drone is airborne");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Landing goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_landing_goal(
  const rclcpp_action::GoalUUID & uuid,
  LandingGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);
  RCLCPP_INFO(this->get_logger(), "Received landing request");
 if (!check_frame_id_global(goal->minimums.header.frame_id)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Landing request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Reach goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_reach_goal(
  const rclcpp_action::GoalUUID & uuid,
  ReachGoalSharedPtr goal)
{
  UNUSED(uuid);
  RCLCPP_INFO(this->get_logger(), "Received reach request");
  if (!(armed_.load(std::memory_order_acquire) &&
    airborne_.load(std::memory_order_acquire)))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Reach request rejected, drone is not airborne");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_frame_id_global(goal->target_pose.header.frame_id)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Reach request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Takeoff goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_takeoff_goal(
  const rclcpp_action::GoalUUID & uuid,
  TakeoffGoalSharedPtr goal)
{
  UNUSED(uuid);
  RCLCPP_INFO(this->get_logger(), "Received takeoff request");
  if (!check_frame_id_global(goal->takeoff_pose.header.frame_id)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Takeoff request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Turn goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FlightControlNode::handle_turn_goal(
  const rclcpp_action::GoalUUID & uuid,
  TurnGoalSharedPtr goal)
{
  UNUSED(uuid);
  RCLCPP_INFO(this->get_logger(), "Received turn request");
  if (!(armed_.load(std::memory_order_acquire) &&
    airborne_.load(std::memory_order_acquire)))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Turn request rejected, drone is not airborne");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_frame_id_global(goal->header.frame_id)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Turn request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Arm cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_arm_cancel(
  const ArmGoalHandleSharedPtr goal_handle)
{
  // Arming cannot be canceled while in progress
  UNUSED(goal_handle);
  RCLCPP_ERROR(
    this->get_logger(),
    "Arming cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Handles a new Disarm cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_disarm_cancel(
  const DisarmGoalHandleSharedPtr goal_handle)
{
  // Disarming cannot be canceled while in progress
  UNUSED(goal_handle);
  RCLCPP_ERROR(
    this->get_logger(),
    "Disarming cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Handles a new Landing cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_landing_cancel(
  const LandingGoalHandleSharedPtr goal_handle)
{
  // Landing cannot be canceled while FMU is being configured
  UNUSED(goal_handle);
  if (de_ascending_.load(std::memory_order_acquire)) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received landing cancellation request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "Landing cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Handles a new Reach cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_reach_cancel(
  const ReachGoalHandleSharedPtr goal_handle)
{
  // Drone must be stopped immediately
  UNUSED(goal_handle);
  RCLCPP_INFO(
    this->get_logger(),
    "Received reach cancellation request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Handles a new Takeoff cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_takeoff_cancel(
  const TakeoffGoalHandleSharedPtr goal_handle)
{
  // Takeoff cannot be canceled while FMU is being configured
  UNUSED(goal_handle);
  if (de_ascending_.load(std::memory_order_acquire)) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received takeoff cancellation request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "Takeoff cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Handles a new Turn cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FlightControlNode::handle_turn_cancel(
  const TurnGoalHandleSharedPtr goal_handle)
{
  // Drone must be stopped immediately
  UNUSED(goal_handle);
  RCLCPP_INFO(
    this->get_logger(),
    "Received turn cancellation request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Starts execution of an Arm.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_arm_accepted(const ArmGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::arm,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Disarm.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_disarm_accepted(const DisarmGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::disarm,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Landing.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_landing_accepted(const LandingGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::landing,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Reach.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_reach_accepted(const ReachGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::reach,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Takeoff.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_takeoff_accepted(const TakeoffGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::takeoff,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Turn.
 *
 * @param goal_handle Handle to the goal object.
 */
void FlightControlNode::handle_turn_accepted(const TurnGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &FlightControlNode::turn,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

} // namespace flight_stack
