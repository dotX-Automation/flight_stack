@# EmPy template for generating <msg>_Subscriber.cpp file.
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
 * @(topic) Subscriber object implementation.
 *
 * Roberto Masocco <robmasocco@@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@@gmail.com>
 *
 * May 13, 2023
 */

#include "@(topic)_Subscriber.hpp"

namespace MicroRTPSAgent
{

/**
 * @@brief Constructor.
 *
 * @@param node Pointer to the ROS 2 node managing this object.
 * @@param outbound_queue Pointer to the outbound message queue.
 * @@param outbound_queue_lk Pointer to the outbound message queue lock.
 * @@param outbound_queue_cv Pointer to the outbound message queue condition variable.
 */
@(topic)_Subscriber::@(topic)_Subscriber(
  rclcpp::Node * node,
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
  std::shared_ptr<std::mutex> outbound_queue_lk,
  std::shared_ptr<std::condition_variable> outbound_queue_cv)
: node_(node),
  topic_id_(@(msgs[0].index(topic) + 1)),
  outbound_queue_(outbound_queue),
  outbound_queue_lk_(outbound_queue_lk),
  outbound_queue_cv_(outbound_queue_cv)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Subscriber::~@(topic)_Subscriber()
{
  subscriber_.reset();
}

/**
 * @@brief Initializes the subscriber.
 */
void @(topic)_Subscriber::init()
{
  // Create and set a standalone callback group for this subscriber
  cgroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = cgroup_;

  // Create the subscriber
  subscriber_ = node_->create_subscription<@(topic)_msg_t>(
    "~/fmu/@(formatted_topic)/in",
    rclcpp::QoS(10),
    [this](const @(topic)_msg_t::SharedPtr msg) -> void
    {
      // Create a new message to be enqueued
      @(topic)_msg_t::SharedPtr msg_ptr(msg);
      OutboundMsg queue_msg;
      queue_msg.topic_id = topic_id_;
      queue_msg.msg = std::static_pointer_cast<void>(msg_ptr);

      // Enqueue the message
      {
        std::lock_guard<std::mutex> lk(*outbound_queue_lk_);
        outbound_queue_->push(queue_msg);
      }
      outbound_queue_cv_->notify_one();
    },
    sub_opts);

  RCLCPP_INFO(node_->get_logger(), "@(topic) subscriber initialized");
}

} // namespace MicroRTPSAgent
