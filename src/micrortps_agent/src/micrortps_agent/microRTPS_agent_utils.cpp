/**
 * MicroRTPS ROS 2 node auxiliary routines.
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

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include <micrortps_agent/microRTPS_agent.hpp>

namespace flight_stack
{

/**
 * @brief Transport type parameter validator.
 *
 * @param p The parameter to validate.
 * @return true if the parameter is valid, false otherwise.
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
 * @brief IMU variance parameter validator.
 *
 * @param p The parameter to validate.
 * @return true if the parameter is valid, false otherwise.
 */
bool AgentNode::validate_imu_variance(const rclcpp::Parameter & p)
{
  // Check that the parameter is a 6-element vector
  if (p.as_double_array().size() != 6) {
    return false;
  }

  // Update entries one by one to avoid copies and memory access violations
  for (int i = 0; i < 6; i++) {
    (*imu_variance_)[i] = p.as_double_array()[i];
  }
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
      this->get_parameter("uart_device").as_string().c_str(),
      this->get_parameter("uart_baudrate").as_int(),
      this->get_parameter("uart_polling_interval").as_int(),
      this->get_parameter("uart_hw_flow_control").as_bool(),
      this->get_parameter("uart_sw_flow_control").as_bool(),
      static_cast<uint8_t>(System::MISSION_COMPUTER),
      this->get_parameter("debug").as_bool());
  } else {
    // Initialize the UDP transport handler
    transporter_ = std::make_shared<UDPTransporter>(
      this->get_parameter("udp_ip_address").as_string().c_str(),
      this->get_parameter("udp_inbound_port").as_int(),
      this->get_parameter("udp_outbound_port").as_int(),
      static_cast<uint8_t>(System::MISSION_COMPUTER),
      this->get_parameter("debug").as_bool());
  }

  if (this->get_parameter("debug").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Transport layer opened");
  }

  if (transporter_->init() < 0) {
    throw std::runtime_error("Transport layer initialization failed");
  }

  if (this->get_parameter("debug").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Transport layer initialized");
  }
}

} // namespace flight_stack
