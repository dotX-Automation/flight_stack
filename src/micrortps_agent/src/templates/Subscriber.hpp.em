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
#include <string>

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>

#include "@(topic)PubSubTypes.h"

#include <rclcpp/rclcpp.hpp>

#include <micrortps_agent/types.hpp>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

using @(topic)_msg_t = px4_msgs::msg::@(topic);
using @(topic)_msg_datatype = px4_msgs::msg::@(topic)PubSubType;

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
  virtual ~@(topic)_Subscriber();
  void init();

  typedef std::shared_ptr<@(topic)_Subscriber> SharedPtr;

private:
  /* ROS 2 node that manages this object. */
  rclcpp::Node * node_;

  /* Topic namespace. */
  std::string ns_;

  /* ID of the topic. */
  uint8_t topic_id_;

  /* FastDDS subscriber data. */
  Participant * mp_participant_;
	Subscriber * mp_subscriber_;
  @(topic)_msg_datatype @(topic)DataType_;

  /* Outbound messages queue, lock and condition variable. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::shared_ptr<std::mutex> outbound_queue_lk_;
  std::shared_ptr<std::condition_variable> outbound_queue_cv_;

  /* FastDDS listener data. */
  class SubListener : public SubscriberListener
	{
	public:
		SubListener() : n_matched(0) {};
		~SubListener() {};
		void onSubscriptionMatched(Subscriber * sub, MatchingInfo & info);
		void onNewDataMessage(Subscriber * sub);
		SampleInfo_t m_info;
		int n_matched;
		@(topic)_msg_t msg;
		uint8_t topic_ID_;
		std::shared_ptr<std::condition_variable> outbound_queue_cv_;
		std::shared_ptr<std::mutex> outbound_queue_lk_;
		std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
	} m_listener_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_
