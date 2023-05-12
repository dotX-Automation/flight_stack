/**
 * MicroRTPS ROS 2 node auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 12, 2023
 */

#include <micrortps_agent/microRTPS_agent.hpp>

namespace MicroRTPSAgent
{

/**
 * @brief Transport type parameter validator.
 *
 * @param p The parameter to validate.
 */
bool AgentNode::validate_transport_type(const rclcpp::Parameter & p)
{
  std::string transport_type = p.as_string();
  if (transport_type != "UART" &&
    transport_type != "uart" &&
    transport_type != "UDP" &&
    transport_type != "udp")
  {
    return false;
  }
  return true;
}

} // namespace MicroRTPSAgent
