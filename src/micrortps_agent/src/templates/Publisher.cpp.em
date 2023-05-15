@# EmPy template for generating <msg>_Publisher.cpp file.
@#
@# Roberto Masocco <robmasocco@gmail.com>
@# Intelligent Systems Lab <isl.torvergata@gmail.com>
@#
@# May 9, 2023
@###############################################################################
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@{
import genmsg.msgs
import re

topic = alias if alias else spec.short_name
formatted_topic = '_'.join([word.lower() for word in re.findall('[A-Z][a-z]*', topic)])
}@
/**
 * @(topic) Publisher object implementation.
 *
 * Roberto Masocco <robmasocco@@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@@gmail.com>
 *
 * May 9, 2023
 */

#include "@(topic)_Publisher.hpp"

namespace MicroRTPSAgent
{

/**
 * @@brief Constructor.
 */
@(topic)_Publisher::@(topic)_Publisher(rclcpp::Node * node)
  : node_(node)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Publisher::~@(topic)_Publisher()
{
  publisher_.reset();
}

/**
 * @@brief Initializes this publisher.
 *
 * @@param node Pointer to the Node object.
 */
void @(topic)_Publisher::init()
{
  publisher_ = node_->create_publisher<@(topic)_msg_t>(
    "~/fmu/@(formatted_topic)/out",
    rclcpp::QoS(10));
  RCLCPP_INFO(node_->get_logger(), "@(topic) publisher initialized");
}

/**
 * @@brief Publishes a message.
 *
 * @@param msg Message to publish.
 */
void @(topic)_Publisher::publish(@(topic)_msg_t & msg)
{
  publisher_->publish(msg);
}

} // namespace MicroRTPSAgent
