@# EmPy template for generating <msg>_Publisher.cpp file.
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
 * @(topic) Publisher object implementation.
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

#include <stdexcept>

#include "@(topic)_Publisher.hpp"

namespace MicroRTPSAgent
{

/**
 * @@brief Constructor.
 */
@(topic)_Publisher::@(topic)_Publisher(
  DomainParticipant * participant,
  rclcpp::Node * node)
: node_(node),
  ns_(node->get_fully_qualified_name()),
  mp_participant_(participant),
  mp_publisher_(nullptr),
  mp_topic_(nullptr),
  mp_writer_(nullptr),
  m_type_(new @(topic)_msg_datatype())
{}

/**
 * @@brief Destructor.
 */
@(topic)_Publisher::~@(topic)_Publisher()
{
  if (mp_writer_ != nullptr) {
    mp_publisher_->delete_datawriter(mp_writer_);
  }
  if (mp_publisher_ != nullptr) {
    mp_participant_->delete_publisher(mp_publisher_);
  }
  if (mp_topic_ != nullptr) {
    mp_participant_->delete_topic(mp_topic_);
  }
}

/**
 * @@brief Initializes this publisher.
 *
 * @@throws RuntimeError if an error occurs in initialization.
 */
void @(topic)_Publisher::init(std::string name, Topic * topic)
{
  // Register the Type
  m_type_.register_type(mp_participant_);

  // Create the Topic
  std::string topic_name = "rt";
	topic_name.append(ns_);
  if (name.empty()) {
    topic_name.append("/fmu/@(formatted_topic)/out");
  } else {
    topic_name.append(name);
  }
  Topic * p_topic = topic;
  if (p_topic == nullptr) {
    mp_topic_ = mp_participant_->create_topic(
      topic_name,
      std::string(m_type_->getName()),
      TOPIC_QOS_DEFAULT);
    p_topic = mp_topic_;
  }
  if (p_topic == nullptr) {
    throw std::runtime_error("@(topic)_Publisher::init: Failed to create topic");
  }

  // Create the Publisher
  mp_publisher_ = mp_participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  if (mp_publisher_ == nullptr) {
    throw std::runtime_error("@(topic)_Publisher::init: Failed to create publisher");
  }

  // Create the DataWriter
  DataWriterQos writer_qos = DATAWRITER_QOS_DEFAULT;
  writer_qos.durability().kind = VOLATILE_DURABILITY_QOS;
  writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  writer_qos.history().kind = KEEP_LAST_HISTORY_QOS;
  writer_qos.history().depth = 10;
  writer_qos.publish_mode().kind = SYNCHRONOUS_PUBLISH_MODE;
  mp_writer_ = mp_publisher_->create_datawriter(p_topic, writer_qos, &m_listener_);
  if (mp_writer_ == nullptr) {
    throw std::runtime_error("@(topic)_Publisher::init: Failed to create data writer");
  }

  RCLCPP_INFO(node_->get_logger(), "@(topic) publisher online");
}

/**
 * @@brief Checks that a new subscription is a match.
 *
 * @@param dw Pointer to the DataWriter.
 * @@param info Matching information.
 */
void @(topic)_Publisher::PubListener::on_publication_matched(DataWriter * dw, const PublicationMatchedStatus & info)
{
  GUID_t remote_guid(info.last_subscription_handle);

  // We support intra-process communication
  bool is_on_same_process = dw->guid().is_on_same_process_as(remote_guid);

	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = true;
	for (size_t i = 0; i < 6; i++) {
		if (dw->guid().guidPrefix.value[i] != remote_guid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint || is_on_same_process) {
		if (info.current_count_change == 1) {
			n_matched_++;
      RCLCPP_INFO(rclcpp::get_logger("DDS Listener"), "@(topic) publisher matched");
		} else if (info.current_count_change == -1) {
			n_matched_--;
			RCLCPP_INFO(rclcpp::get_logger("DDS Listener"), "@(topic) publisher unmatched");
		} else {
      RCLCPP_ERROR(rclcpp::get_logger("DDS Listener"), "@(topic) publisher match error");
    }
	}
}

/**
 * @@brief Publishes a message.
 *
 * @@param msg Message to publish.
 */
void @(topic)_Publisher::publish(@(topic)_msg_t * msg)
{
  mp_writer_->write(msg);
}

} // namespace MicroRTPSAgent
