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

#ifndef MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
#define MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_

#include <atomic>
#include <cstdlib>
#include <memory>
#include <string>

#include "@(topic)PubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastrtps::rtps;

using @(topic)_msg_t = px4_msgs::msg::@(topic);
using @(topic)_msg_datatype = px4_msgs::msg::@(topic)PubSubType;

namespace MicroRTPSAgent
{

class @(topic)_Publisher
{
public:
  @(topic)_Publisher(
    DomainParticipant * participant,
    rclcpp::Node * node);
  virtual ~@(topic)_Publisher();
  void init(std::string name = "", Topic * topic = nullptr);
  void publish(@(topic)_msg_t * msg);

  typedef std::shared_ptr<@(topic)_Publisher> SharedPtr;

private:
  /* ROS 2 node that manages this object. */
  rclcpp::Node * node_;

  /* Namespace of the topic. */
  std::string ns_;

  /* FastDDS publisher data. */
  DomainParticipant * mp_participant_;
	Publisher * mp_publisher_;
  Topic * mp_topic_;
  DataWriter * mp_writer_;
  TypeSupport m_type_;

  /* FastDDS listener data. */
  class PubListener : public DataWriterListener
	{
	public:
		PubListener() : n_matched_(0) {};
		~PubListener() override {};

		void on_publication_matched(DataWriter * dw, const PublicationMatchedStatus & info) override;

		std::atomic_int n_matched_;
	} m_listener_;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__@(topic.upper())_PUBLISHER_HPP_
