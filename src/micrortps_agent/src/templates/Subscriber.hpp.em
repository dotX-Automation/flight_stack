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

#ifndef MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_
#define MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "@(topic)PubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <rclcpp/rclcpp.hpp>

#include <micrortps_agent/types.hpp>

using namespace eprosima::fastdds::dds;
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
    std::shared_ptr<std::condition_variable> outbound_queue_cv,
    uint8_t topic_ID,
    bool localhost_only = false);
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
  bool localhost_only_;
  DomainParticipant * mp_participant_;
  Subscriber * mp_subscriber_;
  DataReader * mp_datareader_;
  Topic * mp_topic_;
  TypeSupport m_type_;

  /* Outbound messages queue, lock and condition variable. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::shared_ptr<std::mutex> outbound_queue_lk_;
  std::shared_ptr<std::condition_variable> outbound_queue_cv_;

  /* FastDDS listener data. */
  class SubListener : public DataReaderListener
	{
	public:
		SubListener() : n_matched_(0) {};
		~SubListener() override {};

		void on_subscription_matched(DataReader * dr, const SubscriptionMatchedStatus & info) override;
		void on_data_available(DataReader * dr) override;

		std::atomic_int n_matched_;
		@(topic)_msg_t msg_;
		uint8_t topic_ID_;

		std::shared_ptr<std::condition_variable> outbound_queue_cv_;
		std::shared_ptr<std::mutex> outbound_queue_lk_;
		std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
	} m_listener_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_SUBSCRIBER_HPP_
