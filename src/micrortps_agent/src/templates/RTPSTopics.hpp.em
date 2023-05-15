@# EmPy template for generating RTPSTopics.hpp file.
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
import os
from px_generate_uorb_topic_files import MsgScope

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/**
 * DDS topics wrapper object definition.
 *
 * Roberto Masocco <robmasocco@@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@@gmail.com>
 *
 * May 13, 2023
 */

#ifndef MICRORTPS_AGENT__RTPSTOPICS_HPP_
#define MICRORTPS_AGENT__RTPSTOPICS_HPP_

#include <condition_variable>
#include <memory>
#include <queue>
#include <type_traits>

#include <fastcdr/Cdr.h>

#include <rclcpp/rclcpp.hpp>

#include <timesync/timesync.hpp>
#include <micrortps_agent/types.hpp>

@[for topic in send_topics]@
#include "@(topic)_Publisher.hpp"
@[end for]@

@[for topic in recv_topics]@
#include "@(topic)_Subscriber.hpp"
@[end for]@

namespace MicroRTPSAgent
{

/**
 * Wraps all DDS data space communications given the configured Bridge topics.
 */
class RTPSTopics
{
public:
  RTPSTopics(
    rclcpp::Node * node,
    std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
    std::shared_ptr<std::mutex> outbound_queue_lk,
    std::shared_ptr<std::condition_variable> outbound_queue_cv,
    bool debug = false);
  ~RTPSTopics();

	template<typename T>
	void sync_timestamp_of_inbound_data(std::shared_ptr<T> msg);
	void publish(const uint8_t topic_ID, char * data_buffer, size_t len);

	template<typename T>
	void sync_timestamp_of_outbound_data(std::shared_ptr<T> msg);
	bool getMsg(const uint8_t topic_ID, MsgSharedPtr msg, eprosima::fastcdr::Cdr & scdr);

  void discardMsg(const uint8_t topic_ID, MsgSharedPtr msg);

  typedef std::shared_ptr<RTPSTopics> SharedPtr;

private:
  /* Debug flag. */
  bool debug_;

  /* Agent ROS 2 node. */
  rclcpp::Node * node_;

  /* Outbound message queue. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::shared_ptr<std::mutex> outbound_queue_lk_;
  std::shared_ptr<std::condition_variable> outbound_queue_cv_;

  /* Time synchronization handler and related publishers. */
  TimeSync::SharedPtr timesync_;
  rclcpp::Publisher<px4_msgs::msg::Timesync>::SharedPtr timesync_fmu_in_ros2_pub_;
  rclcpp::Publisher<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_ros2_pub_;

	/* Publishers, to send inbound data to the ROS 2 data space. */
@[for topic in send_topics]@
	@(topic)_Publisher::SharedPtr @(topic)_pub_;
@[end for]@

	/* Subscribers, to get outbound data from the ROS 2 data space. */
@[for topic in recv_topics]@
	@(topic)_Subscriber::SharedPtr @(topic)_sub_;
@[end for]@

  // SFINAE for ROS 2 message structures
	template<typename T>
  struct hasTimestampSample
  {
  private:
    template<typename U>
    static constexpr auto detect(int) -> decltype(std::declval<U>().timestamp_sample, std::true_type{});

		template<typename U>
		static std::false_type detect(...);
	public:
		static constexpr bool value = decltype(detect<T>(0))::value;
  };

  /* ROS 2 message metadata getters. */
  template<class T>
	inline uint64_t getMsgTimestamp(const std::shared_ptr<const T> msg) {return msg->timestamp;}

	template<typename T>
	inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const std::shared_ptr<const T> msg) {return msg->timestamp_sample;}

  template<typename T>
	inline typename std::enable_if<!hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const std::shared_ptr<const T> msg)
  {
    (void)msg;
    return 0;
  }

	template<class T>
	inline uint8_t getMsgSysID(const std::shared_ptr<const T> msg) {return msg->sys_id;}

	template<class T>
	inline uint8_t getMsgSeq(const std::shared_ptr<const T> msg) {return msg->seq;}

  template<class T>
	inline uint64_t getMsgTimestampSample(const std::shared_ptr<const T> msg) {return getMsgTimestampSample_impl(msg);}

  /* ROS 2 message metadata setters. */
  template<class T>
	inline void setMsgTimestamp(std::shared_ptr<T> msg, const uint64_t & timestamp) {msg->set__timestamp(timestamp);}

	template<class T>
	inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(std::shared_ptr<T> msg, const uint64_t & timestamp_sample) {msg->set__timestamp_sample(timestamp_sample);}

  template<typename T>
	inline typename std::enable_if<!hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(std::shared_ptr<T> msg, const uint64_t & timestamp_sample)
  {
    (void)msg;
    (void)timestamp_sample;
  }

	template<class T>
	inline void setMsgSysID(std::shared_ptr<T> msg, const uint8_t & sys_id) {msg->set__sys_id(sys_id);}

	template<class T>
	inline void setMsgSeq(std::shared_ptr<T> msg, const uint8_t & seq) {msg->set__seq(seq);}

  template<class T>
	inline void setMsgTimestampSample(std::shared_ptr<T> msg, const uint64_t & timestamp_sample) {setMsgTimestampSample_impl(msg, timestamp_sample);}
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__RTPSTOPICS_HPP_
