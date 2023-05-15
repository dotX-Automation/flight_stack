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
import genmsg.msgs
import re

topic = alias if alias else spec.short_name
formatted_topic = '_'.join([word.lower() for word in re.findall('[A-Z][a-z]*', topic)])
}@
/**
 * @(topic) Publisher object definition.
 *
 * Roberto Masocco <robmasocco@@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@@gmail.com>
 *
 * May 9, 2023
 */

#ifndef MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
#define MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "@(topic)PubSubTypes.h"

#include <rclcpp/rclcpp.hpp>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

using @(topic)_msg_t = px4_msgs::msg::@(topic);
using @(topic)_msg_datatype = px4_msgs::msg::@(topic)PubSubType;

namespace MicroRTPSAgent
{

class @(topic)_Publisher
{
public:
  @(topic)_Publisher(rclcpp::Node * node, const std::string & ns);
  virtual ~@(topic)_Publisher();
  void init();
  void publish(@(topic)_msg_t * msg);

  typedef std::shared_ptr<@(topic)_Publisher> SharedPtr;

private:
  /* ROS 2 node that manages this object. */
  rclcpp::Node * node_;

  /* Namespace of the topic. */
  std::string ns_;

  /* FastDDS publisher data. */
  Participant * mp_participant_;
	Publisher * mp_publisher_;
  @(topic)_msg_datatype @(topic)DataType_;

  /* FastDDS listener data. */
  class PubListener : public PublisherListener
	{
	public:
		PubListener() : n_matched(0) {};
		~PubListener() {};
		void onPublicationMatched(Publisher * pub, MatchingInfo & info);
		int n_matched;
	} m_listener_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
