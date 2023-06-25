/**
 * Flight Control service callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 26, 2022
 */

#define UNUSED(arg) (void)(arg)

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Reboots the FMU.
 *
 * @param req Service request to parse.
 * @param resp Service response to fill.
 */
void FlightControlNode::reboot_callback(
  Trigger::Request::SharedPtr req,
  Trigger::Response::SharedPtr resp)
{
  UNUSED(req);

  send_fmu_command(VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0f);
  resp->set__success(true);
  resp->set__message("");
  RCLCPP_WARN(this->get_logger(), "FMU reboot issued");
}

/**
 * @brief Resets the internal state of the module.
 *
 * @param req Service request to parse.
 * @param resp Service response to fill.
 */
void FlightControlNode::reset_callback(
  Trigger::Request::SharedPtr req,
  Trigger::Response::SharedPtr resp)
{
  UNUSED(req);

  {
    std::unique_lock<std::mutex> operation_lk(operation_lock_);

    // Reset synchronization primitives
    fmu_cmd_ack_received_.store(true, std::memory_order_release);
    fmu_cmd_success_.store(false, std::memory_order_release);
    takeoff_status_received_.store(true, std::memory_order_release);

    // Reset internal state variables
    armed_.store(false, std::memory_order_release);
    airborne_.store(false, std::memory_order_release);
    de_ascending_.store(false, std::memory_order_release);
    drone_pose_ = PoseKit::DynamicPose{};
    last_stream_ts_.store(0ULL, std::memory_order_release);
    last_takeoff_status_ = TakeoffStatus::TAKEOFF_STATE_UNINITIALIZED;
    low_battery_ = false;
    low_battery_timer_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    last_pose_timestamp_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

    // Reset setpoints data
    setpoint_lock_.lock();
    fmu_setpoint_ = Setpoint{};
    setpoint_lock_.unlock();
  }

  resp->set__success(true);
  resp->set__message("");

  RCLCPP_WARN(this->get_logger(), "RESET");
}

/**
 * @brief Toggles the setpoints publishing timer on demand.
 *
 * @param req Service request to parse.
 * @param resp Service response to fill.
 */
void FlightControlNode::setpoints_switch_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  {
    std::unique_lock<std::mutex> operation_lk(operation_lock_);

    if ((req->data) && (setpoints_timer_ == nullptr)) {
      // Request to activate timer
      activate_setpoints_timer();
      resp->set__success(true);
      resp->set__message("");
      return;
    }
    if (!(req->data) && (setpoints_timer_ != nullptr)) {
      // Request to deactivate timer
      deactivate_setpoints_timer();
      resp->set__success(true);
      resp->set__message("");
      return;
    }
  }
  resp->set__success(false);
  resp->set__message("Invalid request");
}

} // namespace FlightControl
