/**
 * MicroRTPS ROS 2 node auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 12, 2023
 */

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

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

  transport_type_ = transport_type;
  std::transform(
    transport_type_.begin(),
    transport_type_.end(),
    transport_type_.begin(),
    [](unsigned char c) {return std::tolower(c);});

  return true;
}

/**
 * @brief Initializes the transport handler.
 *
 * @throws RuntimeError if transport layer initialization fails.
 */
void AgentNode::init_transporter()
{
  if (transport_type_ == "uart") {
    // Initialize the UART transport handler
    transporter_ = std::make_shared<UARTTransporter>(
      this->get_parameter("uart_device").as_string(),
      this->get_parameter("uart_baudrate").as_int(),
      this->get_parameter("uart_polling_interval").as_int(),
      this->get_parameter("uart_hw_flow_control").as_bool(),
      this->get_parameter("uart_sw_flow_control").as_bool(),
      System::MISSION_COMPUTER,
      this->get_parameter("debug").as_bool());
  } else {
    // Initialize the UDP transport handler
    transporter_ = std::make_shared<UDPTransporter>(
      this->get_parameter("udp_ip_address").as_string(),
      this->get_parameter("udp_inbound_port").as_int(),
      this->get_parameter("udp_outbound_port").as_int(),
      System::MISSION_COMPUTER,
      this->get_parameter("debug").as_bool());
  }

  if (transporter_->init() < 0) {
    throw std::runtime_error("Transport layer initialization failed");
  }
}

} // namespace MicroRTPSAgent
