/**
 * Flight Control timer callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 26, 2022
 */

#define NOOP ((void)0)

#include <flight_control/flight_control.hpp>

namespace FlightControl
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
    if (now - last_stream_ts_.load(std::memory_order_acquire) > setpoint_stream_timeout_us_) {
      stop_drone();
      last_stream_ts_.store(0ULL, std::memory_order_release);
      RCLCPP_WARN(this->get_logger(), "Setpoint stream timed out, holding position");
    } else {
      return;
    }
  }

  OffboardControlMode control_mode_msg{};
  TrajectorySetpoint setpoint_msg{};
  std::array<float, 3> nans{NAN, NAN, NAN};
  uint64_t timestamp = get_time_us();

  // Get current setpoint
  setpoint_lock_.lock();
  Setpoint current_setpoint = fmu_setpoint_;
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
    // Should never happen
    RCLCPP_FATAL(this->get_logger(), "Invalid OFFBOARD control mode stored");
    throw std::runtime_error("Invalid OFFBOARD control mode stored");
  }

  // Fill trajectory_setpoint message (from NWU to NED)
  setpoint_msg.set__timestamp(timestamp);
  setpoint_msg.set__acceleration(nans);
  setpoint_msg.set__jerk(nans);
  setpoint_msg.set__thrust(nans);
  if (current_setpoint.control_mode == ControlModes::POSITION) {
    setpoint_msg.set__x(current_setpoint.x);
    setpoint_msg.set__y(-current_setpoint.y);
    setpoint_msg.set__z(-current_setpoint.z);
    setpoint_msg.set__yaw(-current_setpoint.yaw);
    setpoint_msg.set__vx(NAN);
    setpoint_msg.set__vy(NAN);
    setpoint_msg.set__vz(NAN);
    setpoint_msg.set__yawspeed(NAN);
  }
  if (current_setpoint.control_mode == ControlModes::VELOCITY) {
    setpoint_msg.set__x(NAN);
    setpoint_msg.set__y(NAN);
    setpoint_msg.set__z(NAN);
    setpoint_msg.set__yaw(-current_setpoint.yaw);
    setpoint_msg.set__vx(current_setpoint.vx);
    setpoint_msg.set__vy(-current_setpoint.vy);
    setpoint_msg.set__vz(-current_setpoint.vz);
    setpoint_msg.set__yawspeed(-current_setpoint.vyaw);
  }

  // Publish messages
  offboard_control_mode_pub_->publish(control_mode_msg);
  trajectory_setpoint_pub_->publish(setpoint_msg);
}

/**
 * @brief Updates tf2 transforms.
 */
void FlightControlNode::tf_timer_callback()
{
  TransformStamped map_to_odom{};

  // Start listening
  // map -> odom
  try {
    map_to_odom = tf_buffer_->lookupTransform(
      odom_frame_,
      map_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    map_to_odom_ = map_to_odom;
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }
}

} // namespace FlightControl
