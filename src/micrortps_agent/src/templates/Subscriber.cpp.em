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

#include <fastrtps/Domain.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

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
  ns_(node->get_fully_qualified_name()),
  topic_id_(@(msgs[0].index(topic) + 1)),
  mp_participant_(nullptr),
  mp_subscriber_(nullptr),
  outbound_queue_(outbound_queue),
  outbound_queue_lk_(outbound_queue_lk),
  outbound_queue_cv_(outbound_queue_cv)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Subscriber::~@(topic)_Subscriber()
{
  Domain::removeParticipant(mp_participant_);
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

  // Create the participant
  ParticipantAttributes PParam;
  PParam.domainId = 0;
  PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
  PParam.rtps.builtin.writerHistoryMemoryPolicy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  std::string nodeName = ns_;
	nodeName.append("/@(topic)_subscriber");
	PParam.rtps.setName(nodeName.c_str());
  mp_participant_ = Domain::createParticipant(PParam);
	if (mp_participant_ == nullptr) {
		throw std::runtime_error("@(topic)_Subscriber::init: Failed to create participant");
	}

  // Register the type
	Domain::registerType(mp_participant_, static_cast<TopicDataType *>(&@(topic)DataType_));

  // Create the subscriber
  SubscriberAttributes Rparam;
	Rparam.topic.topicKind = NO_KEY;
	Rparam.topic.topicDataType = @(topic)DataType_.getName();
  std::string topicName = "rt";
	topicName.append(ns_);
  topicName.append("/fmu/@(formatted_topic)/in");
  Rparam.topic.topicName = topicName;
  mp_subscriber_ = Domain::createSubscriber(
    mp_participant_,
    Rparam,
    static_cast<SubscriberListener *>(&m_listener_));
  if (mp_subscriber_ == nullptr) {
		throw std::runtime_error("@(topic)_Subscriber::init: Failed to create subscriber");
	}

  RCLCPP_INFO(node_->get_logger(), "@(topic) subscriber initialized");
}

/**
 * @@brief Checks that a new publisher is a match.
 *
 * @@param pub Pointer to the subscriber.
 * @@param info Matching information.
 */
void @(topic)_Subscriber::SubListener::onSubscriptionMatched(Subscriber * sub, MatchingInfo & info)
{
@# Since the Timesync runs on the bridge itself, it is required that there is a
@# match between two topics of the same entity
@[if topic != 'Timesync' and topic != 'timesync' and topic != 'TimesyncStatus' and topic != 'timesync_status']@
	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = false;

	for (size_t i = 0; i < 6; i++) {
		if (sub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint) {
		if (info.status == MATCHED_MATCHING) {
			n_matched++;
      RCLCPP_INFO(rclcpp::get_logger("RTPS Listener"), "@(topic) subscriber matched");
		} else {
			n_matched--;
      RCLCPP_INFO(rclcpp::get_logger("RTPS Listener"), "@(topic) subscriber unmatched");
		}
	}

@[else]@
  // This is silent, since it's intended
	(void)sub;

	if (info.status == MATCHED_MATCHING) {
		n_matched++;
	} else {
		n_matched--;
	}
@[end if]@
}

/**
 * @@brief Callback for new data.
 *
 * @@param sub Pointer to the subscriber.
 */
void @(topic)_Subscriber::SubListener::onNewDataMessage(Subscriber * sub)
{
	if (n_matched > 0) {
		// Take data from the transport layer and enqueue it
		if (sub->takeNextData(&msg, &m_info)) {
			if (m_info.sampleKind == ALIVE) {
        // Create a new message to be enqueued
				@(topic)_msg_t * msg_ptr = new @(topic)_msg_t(msg);
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
