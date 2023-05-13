@# EmPy template for generating <msg>_Publisher.hpp file.
@#
@# Roberto Masocco <robmasocco@gmail.com>
@# Intelligent Systems Lab <isl.torvergata@gmail.com>
@#
@# May 9, 2023
@###############################################################################
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@{
import re

topic = alias if alias else spec.short_name
formatted_topic = '_'.join([word.lower() for word in re.findall('[A-Z][a-z]*', topic)])
}@
/**
 * @(topic) Publisher object definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 9, 2023
 */

#ifndef MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
#define MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/@(formatted_topic).hpp>

using @(topic)_msg_t = px4_msgs::msg::@(topic);

namespace MicroRTPSAgent
{

class @(topic)_Publisher
{
public:
  @(topic)_Publisher(rclcpp::Node * node);
  ~@(topic)_Publisher();
  void init();
  void publish(@(topic)_msg_t & msg);

  typedef std::shared_ptr<@(topic)_Publisher> SharedPtr;

  inline rclcpp::Publisher<@(topic)_msg_t>::SharedPtr get_publisher() {return publisher_;}

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<@(topic)_msg_t>::SharedPtr publisher_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
