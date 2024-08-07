@# EmPy template for generating RTPSTopics.hpp file.
@#
@# Roberto Masocco <r.masocco@dotxautomation.com>
@#
@# June 24, 2024
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
 * Roberto Masocco <r.masocco@@dotxautomation.com>
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

#include <array>
#include <condition_variable>
#include <memory>
#include <queue>
#include <type_traits>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
#include <fastcdr/exceptions/NotEnoughMemoryException.h>

#include <rclcpp/rclcpp.hpp>

#include <micrortps_agent/types.hpp>

#include <dua_qos_cpp/dua_qos.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <px4_msgs/msg/vehicle_local_position_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include "timesync.hpp"

@[for topic in send_topics]@
#include "@(topic)_Publisher.hpp"
@[end for]@

@[for topic in recv_topics]@
#include "@(topic)_Subscriber.hpp"
@[end for]@

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

@[for topic in (recv_topics + send_topics)]@
using @(topic)_msg_t = px4_msgs::msg::@(topic);
@[end for]@

namespace flight_stack
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
    std::string frame_prefix,
    std::shared_ptr<std::array<double, 6>> imu_variance,
    bool debug = false,
    bool localhost_only = false);
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
  /* Configuration variables. */
  bool debug_;
  std::string frame_prefix_;
  std::shared_ptr<std::array<double, 6>> imu_variance_;
  bool localhost_only_;

  /* Agent DDS Participant for the bridge. */
  DomainParticipant * participant_;

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

  /* Publishers for converted data. */
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionStamped>::SharedPtr vehicle_local_position_stamped_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeStamped>::SharedPtr vehicle_attitude_stamped_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

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

} // namespace flight_stack

#endif // MICRORTPS_AGENT__RTPSTOPICS_HPP_
