@# EmPy template for generating <msg>_Subscriber.hpp file.
@#
@# Roberto Masocco <robmasocco@gmail.com>
@# Intelligent Systems Lab <isl.torvergata@gmail.com>
@#
@# May 13, 2023
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
 * @(topic) Subscriber object definition.
 *
 * Roberto Masocco <robmasocco@@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@@gmail.com>
 *
 * May 13, 2023
 */

#ifndef MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_
#define MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/@(formatted_topic).hpp>

#include <micrortps_agent/types.hpp>

using @(topic)_msg_t = px4_msgs::msg::@(topic);

namespace MicroRTPSAgent
{

class @(topic)_Subscriber
{
public:
  @(topic)_Subscriber(
    rclcpp::Node * node,
    std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
    std::shared_ptr<std::mutex> outbound_queue_lk,
    std::shared_ptr<std::condition_variable> outbound_queue_cv);
  ~@(topic)_Subscriber();
  void init();

  inline rclcpp::Subscription<@(topic)_msg_t>::SharedPtr get_subscriber() {return subscriber_;}

  typedef std::shared_ptr<@(topic)_Subscriber> SharedPtr;

private:
  /* ROS 2 node that manages this object. */
  rclcpp::Node * node_;

  /* ROS 2 subscriber. */
  rclcpp::CallbackGroup::SharedPtr cgroup_;
  rclcpp::Subscription<@(topic)_msg_t>::SharedPtr subscriber_;

  /* ID of the topic. */
  uint8_t topic_id_;

  /* Outbound messages queue, lock and condition variable. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::shared_ptr<std::mutex> outbound_queue_lk_;
  std::shared_ptr<std::condition_variable> outbound_queue_cv_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_
