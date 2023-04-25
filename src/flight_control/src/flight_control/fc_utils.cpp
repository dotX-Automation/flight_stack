/**
 * Flight control module auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 24, 2022
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
 * @brief Routine to initialize a spinlock.
 *
 * @param lock Pointer to the lock to initialize.
 *
 * @throws RuntimeError
 */
void FlightControlNode::create_spinlock(pthread_spinlock_t * lock)
{
  if (pthread_spin_init(lock, PTHREAD_PROCESS_PRIVATE)) {
    throw std::runtime_error("Failed to initialize spinlock");
  }
}

/**
 * @brief Routine to declare a boolean node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void FlightControlNode::declare_bool_parameter(
  std::string && name,
  bool default_val,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_BOOL);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a 64-bit floating point node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Floating point range initial value.
 * @param to Floating point range final value.
 * @param step Floating point range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void FlightControlNode::declare_double_parameter(
  std::string && name,
  double default_val, double from, double to, double step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_DOUBLE);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare an integer node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Integer range initial value.
 * @param to Integer range final value.
 * @param step Integer range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void FlightControlNode::declare_int_parameter(
  std::string && name,
  int64_t default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_INTEGER);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__integer_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

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
  std::this_thread::sleep_for(std::chrono::seconds(1)); // Let PX4 adapt
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
bool FlightControlNode::change_setpoint(const Setpoint & new_setpoint)
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
  if ((new_setpoint.yaw != NAN) &&
    ((new_setpoint.yaw < -M_PIf32) || (new_setpoint.yaw > M_PIf32)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: yaw out of range");
    return false;
  }
  // Check target coordinates
  if ((new_setpoint.control_mode == ControlModes::POSITION) &&
    ((new_setpoint.x == NAN) || (new_setpoint.y == NAN) || (new_setpoint.z == NAN) ||
    (new_setpoint.z > 0.0f) ||
    (new_setpoint.yaw == NAN)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid position coordinates");
    return false;
  }
  // Check linear velocities
  if ((new_setpoint.control_mode == ControlModes::VELOCITY) &&
    ((new_setpoint.vx == NAN) || (new_setpoint.vy == NAN) || (new_setpoint.vz == NAN)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid linear velocities");
    return false;
  }
  // Check VELOCITY yaw configuration
  if ((new_setpoint.control_mode == ControlModes::VELOCITY) &&
    ((new_setpoint.yaw == NAN) && (new_setpoint.vyaw == NAN)))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid new setpoint: invalid yaw configuration");
    return false;
  }

  // Update stored setpoint
  pthread_spin_lock(&(this->setpoint_lock_));
  fmu_setpoint_ = new_setpoint;
  pthread_spin_unlock(&(this->setpoint_lock_));

  return true;
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
  msg.set__timestamp(fmu_timestamp_.load(std::memory_order_acquire));

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
 * @brief Stops the drone at the given position.
 *
 * @param pose Stop pose.
 */
void FlightControlNode::stop_drone(const DronePose & pose)
{
  change_setpoint(
    Setpoint(
      pose.position(0),
      pose.position(1),
      pose.position(2),
      pose.rpy.gamma()));
}

/**
 * @brief Checks if the drone is currently stabilized.
 *
 * @param pose Current drone pose.
 *
 * @return Yes or no.
 */
bool FlightControlNode::is_stabilized(const DronePose & pose)
{
  float rp_confidence = roll_pitch_stabilization_confidence_;
  float vh_max = v_horz_stabilization_max_;
  float vv_max = v_vert_stabilization_max_;
  if ((abs(pose.rpy.alpha()) > rp_confidence) ||
    (abs(pose.rpy.beta()) > rp_confidence) ||
    (abs(pose.velocity(0)) > vh_max) ||
    (abs(pose.velocity(1)) > vh_max) ||
    (abs(pose.velocity(2)) > vv_max))
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
  const Eigen::Vector3f & current,
  const Eigen::Vector3f & target,
  float confidence_radius)
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
bool FlightControlNode::is_oriented(float yaw, float tgt_yaw)
{
  float diff = abs(tgt_yaw - yaw);
  if (diff > M_PIf32) {
    diff = abs((2.0f * M_PIf32) - diff);
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
float FlightControlNode::get_distance(
  const Eigen::Vector3f & pos1,
  const Eigen::Vector3f & pos2)
{
  Eigen::Vector3f distance = pos1 - pos2;
  return distance.norm();
}

} // namespace FlightControl
