/**
 * Flight Control service callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 26, 2022
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

#define UNUSED(arg) (void)(arg)

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Resets the internal state of the module.
 *
 * @param req Service request to parse.
 * @param resp Service response to populate.
 */
void FlightControlNode::reset_callback(
  Reset::Request::SharedPtr req,
  Reset::Response::SharedPtr resp)
{
  UNUSED(req);

  //! This routine doesn't check anything, and doesn't take any lock.
  //! Use this service with caution only during testing as an override!

  // Reset synchronization primitives
  fmu_cmd_ack_received_.store(true, std::memory_order_release);
  fmu_cmd_success_.store(false, std::memory_order_release);
  takeoff_status_received_.store(true, std::memory_order_release);

  // Reset internal state variables
  armed_.store(false, std::memory_order_release);
  airborne_.store(false, std::memory_order_release);
  drone_pose_ = DronePose{};
  last_takeoff_status_ = TakeoffStatus::TAKEOFF_STATE_UNINITIALIZED;
  low_battery_ = false;
  low_battery_timer_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  last_pose_timestamp_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  // Reset setpoints data
  fmu_setpoint_ = Setpoint{};

  // Reset FMU timestamp
  fmu_timestamp_.store(0L, std::memory_order_release);

  resp->result.set__result(true);
  resp->result.set__error_msg("");

  RCLCPP_WARN(this->get_logger(), "%s RESET", this->get_name());
}

/**
 * @brief Toggles the setpoints publishing timer on demand.
 *
 * @param req Service request to parse.
 * @param resp Service response to populate.
 */
void FlightControlNode::setpoints_switch_callback(
  SetpointsSwitch::Request::SharedPtr req,
  SetpointsSwitch::Response::SharedPtr resp)
{
  //! This routine doesn't check anything, and doesn't take any lock.
  //! Use this service with caution only during testing as an override!

  if ((req->stream_active == SetpointsSwitch::Request::ON) &&
    (setpoints_timer_ == nullptr))
  {
    // Request to activate timer
    activate_setpoints_timer();
    resp->result.set__result(CommandResult::SUCCESS);
    resp->result.set__error_msg("");
    return;
  }
  if ((req->stream_active == SetpointsSwitch::Request::OFF) &&
    (setpoints_timer_ != nullptr))
  {
    // Request to deactivate timer
    deactivate_setpoints_timer();
    resp->result.set__result(CommandResult::SUCCESS);
    resp->result.set__error_msg("");
    return;
  }
  resp->result.set__result(CommandResult::FAILED);
  resp->result.set__error_msg("Invalid request");
}

} // namespace FlightControl
