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
  DomainParticipant * participant,
  rclcpp::Node * node,
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
  std::shared_ptr<std::mutex> outbound_queue_lk,
  std::shared_ptr<std::condition_variable> outbound_queue_cv,
  uint8_t topic_ID)
: node_(node),
  ns_(node->get_fully_qualified_name()),
  topic_id_(topic_ID),
  mp_participant_(participant),
  mp_subscriber_(nullptr),
  mp_datareader_(nullptr),
  mp_topic_(nullptr),
  m_type_(new @(topic)_msg_datatype()),
  outbound_queue_(outbound_queue),
  outbound_queue_lk_(outbound_queue_lk),
  outbound_queue_cv_(outbound_queue_cv)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Subscriber::~@(topic)_Subscriber()
{
  if (mp_datareader_ != nullptr) {
    mp_subscriber_->delete_datareader(mp_datareader_);
  }
  if (mp_topic_ != nullptr) {
    mp_participant_->delete_topic(mp_topic_);
  }
  if (mp_subscriber_ != nullptr) {
    mp_participant_->delete_subscriber(mp_subscriber_);
  }
}

/**
 * @@brief Initializes the subscriber.
 *
 * @@throws RuntimeError if something fails during initialization.
 */
void @(topic)_Subscriber::init()
{
  // Configure the listener
  m_listener_.topic_ID_ = topic_id_;
	m_listener_.outbound_queue_cv_ = outbound_queue_cv_;
	m_listener_.outbound_queue_lk_ = outbound_queue_lk_;
	m_listener_.outbound_queue_ = outbound_queue_;

  // Register the Type
  m_type_.register_type(mp_participant_);

  // Create the Topic
  std::string topic_name = "rt";
	topic_name.append(ns_);
  topic_name.append("/fmu/@(formatted_topic)/in");
  mp_topic_ = mp_participant_->create_topic(
    topic_name,
    std::string(m_type_->getName()),
    TOPIC_QOS_DEFAULT);
  if (mp_topic_ == nullptr) {
    throw std::runtime_error("@(topic)_Subscriber::init: Failed to create topic");
  }

  // Create the Subscriber
  mp_subscriber_ = mp_participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
  if (mp_subscriber_ == nullptr) {
    throw std::runtime_error("@(topic)_Subscriber::init: Failed to create subscriber");
  }

  // Create the DataReader
  DataReaderQos reader_qos = DATAREADER_QOS_DEFAULT;
  reader_qos.durability().kind = VOLATILE_DURABILITY_QOS;
  reader_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  reader_qos.history().kind = KEEP_LAST_HISTORY_QOS;
  reader_qos.history().depth = 10;
  reader_qos.endpoint().history_memory_policy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  mp_datareader_ = mp_subscriber_->create_datareader(mp_topic_, reader_qos, &m_listener_);
  if (mp_datareader_ == nullptr) {
    throw std::runtime_error("@(topic)_Subscriber::init: Failed to create data reader");
  }

  RCLCPP_INFO(node_->get_logger(), "@(topic) subscriber online");
}

/**
 * @@brief Checks that a new publisher is a match.
 *
 * @@param dr Pointer to the DataReader.
 * @@param info Matching information.
 */
void @(topic)_Subscriber::SubListener::on_subscription_matched(DataReader * dr, const SubscriptionMatchedStatus & info)
{
  GUID_t remote_guid(info.last_publication_handle);

  // We support intra-process communication
  bool is_on_same_process = dr->guid().is_on_same_process_as(remote_guid);

	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = true;
	for (size_t i = 0; i < 6; i++) {
		if (dr->guid().guidPrefix.value[i] != remote_guid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint || is_on_same_process) {
		if (info.current_count_change == 1) {
			n_matched_++;
      RCLCPP_INFO(rclcpp::get_logger("DDS Listener"), "@(topic) subscriber matched");
		} else if (info.current_count_change == -1) {
			n_matched_--;
			RCLCPP_INFO(rclcpp::get_logger("DDS Listener"), "@(topic) subscriber unmatched");
		} else {
      RCLCPP_ERROR(rclcpp::get_logger("DDS Listener"), "@(topic) subscriber match error");
    }
	}
}

/**
 * @@brief Callback for new data.
 *
 * @@param dr Pointer to the DataReader.
 */
void @(topic)_Subscriber::SubListener::on_data_available(DataReader * dr)
{
  SampleInfo info;
	if (n_matched_ > 0) {
		// Take data from the transport layer and enqueue it
		if (dr->take_next_sample(&msg_, &info) == ReturnCode_t::RETCODE_OK) {
			if (info.valid_data) {
        // Create a new message to be enqueued
				@(topic)_msg_t * msg_ptr = new @(topic)_msg_t(msg_);
        OutboundMsg queue_msg;
        queue_msg.topic_id = topic_ID_;
        queue_msg.msg = static_cast<void *>(msg_ptr);

        // Enqueue the message
        {
          std::lock_guard<std::mutex> lk(*outbound_queue_lk_);
          outbound_queue_->push(queue_msg);
        }
        outbound_queue_cv_->notify_one();
			}
		}
	}
}

} // namespace MicroRTPSAgent
