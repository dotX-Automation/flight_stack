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

/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MICRORTPS_AGENT__RTPSTOPICS_HPP_
#define MICRORTPS_AGENT__RTPSTOPICS_HPP_

#include <condition_variable>
#include <memory>
#include <queue>
#include <type_traits>

#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
#include <fastcdr/exceptions/NotEnoughMemoryException.h>

#include <rclcpp/rclcpp.hpp>

#include <micrortps_agent/types.hpp>

#include "timesync.hpp"

@[for topic in send_topics]@
#include "@(topic)_Publisher.hpp"
@[end for]@

@[for topic in recv_topics]@
#include "@(topic)_Subscriber.hpp"
@[end for]@

@[for topic in (recv_topics + send_topics)]@
using @(topic)_msg_t = px4_msgs::msg::@(topic);
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
	void sync_timestamp_of_inbound_data(T & msg);
	void publish(const uint8_t topic_ID, char * data_buffer, size_t len);

	template<typename T>
	void sync_timestamp_of_outbound_data(T * msg);
	bool getMsg(const uint8_t topic_ID, void * msg, eprosima::fastcdr::Cdr & scdr);

  void discardMsg(const uint8_t topic_ID, void * msg);

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

  /* Timesync handler. */
  TimeSync::SharedPtr _timesync;

	/* Publishers, to send inbound data to the ROS 2 data space. */
@[for topic in send_topics]@
@[    if topic == 'Timesync' or topic == 'timesync']@
	@(topic)_Publisher::SharedPtr timesync_pub_;
	@(topic)_Publisher::SharedPtr timesync_fmu_in_pub_;
@[    elif topic == 'TimesyncStatus' or topic == 'timesync_status']@
  @(topic)_Publisher::SharedPtr timesync_status_pub_;
@[    else]@
	@(topic)_Publisher::SharedPtr @(topic)_pub_;
@[    end if]@
@[end for]@

	/* Subscribers, to get outbound data from the ROS 2 data space. */
@[for topic in recv_topics]@
	@(topic)_Subscriber::SharedPtr @(topic)_sub_;
@[end for]@

  // SFINAE for message structures
	template<typename T>
  struct hasTimestampSample
  {
	private:
		template<typename U, typename = decltype(std::declval<U>().timestamp_sample(int64_t()))>
		static std::true_type detect(int);
		template<typename U>
		static std::false_type detect(...);
	public:
		static constexpr bool value = decltype(detect<T>(0))::value;
  };

  /* Message metadata getters. */
  template<class T>
	inline uint64_t getMsgTimestamp(const T * msg) {return msg->timestamp();}

	template<typename T>
	inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const T * msg) {return msg->timestamp_sample();}

  template<typename T>
	inline typename std::enable_if<!hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const T *) {return 0;}

	template<class T>
	inline uint8_t getMsgSysID(const T * msg) {return msg->sys_id();}

	template<class T>
	inline uint8_t getMsgSeq(const T * msg) {return msg->seq();}

  template<class T>
	inline uint64_t getMsgTimestampSample(const T * msg) {return getMsgTimestampSample_impl(msg);}

  /* Message metadata setters. */
  template<class T>
	inline void setMsgTimestamp(T * msg, const uint64_t & timestamp) {msg->timestamp() = timestamp;}

	template<class T>
	inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(T * msg, const uint64_t & timestamp_sample) {msg->timestamp_sample() = timestamp_sample;}

  template<typename T>
	inline typename std::enable_if<!hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(T *, const uint64_t &) {}

	template<class T>
	inline void setMsgSysID(T * msg, const uint8_t & sys_id) {msg->sys_id() = sys_id;}

	template<class T>
	inline void setMsgSeq(T * msg, const uint8_t & seq) {msg->seq() = seq;}

  template<class T>
	inline void setMsgTimestampSample(T * msg, const uint64_t & timestamp_sample) {setMsgTimestampSample_impl(msg, timestamp_sample);}
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__RTPSTOPICS_HPP_
