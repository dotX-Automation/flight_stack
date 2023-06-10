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

#include <fastrtps/Domain.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

namespace MicroRTPSAgent
{

/**
 * @@brief Constructor.
 */
@(topic)_Publisher::@(topic)_Publisher(rclcpp::Node * node)
: node_(node),
  ns_(node->get_fully_qualified_name()),
  mp_participant_(nullptr),
  mp_publisher_(nullptr)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Publisher::~@(topic)_Publisher()
{
  Domain::removeParticipant(mp_participant_);
}

/**
 * @@brief Initializes this publisher.
 *
 * @@throws RuntimeError if an error occurs in initialization.
 */
void @(topic)_Publisher::init(std::string name)
{
  // Create RTPSParticipant
	ParticipantAttributes PParam;
  PParam.domainId = 0;
  PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
  PParam.rtps.builtin.writerHistoryMemoryPolicy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

  // Set participant name
  std::string nodeName = ns_;
	nodeName.append("/@(topic)_publisher");
	PParam.rtps.setName(nodeName.c_str());

  mp_participant_ = Domain::createParticipant(PParam);
  if (mp_participant_ == nullptr) {
		throw std::runtime_error("@(topic)_Publisher::init: Failed to create participant");
	}

  // Register the type
	Domain::registerType(mp_participant_, static_cast<TopicDataType *>(&@(topic)DataType_));

  // Create publisher
	PublisherAttributes Wparam;
	Wparam.topic.topicKind = NO_KEY;
	Wparam.topic.topicDataType = @(topic)DataType_.getName();
  Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
  std::string topicName = "rt";
	topicName.append(ns_);
  if (name.empty()) {
    topicName.append("/fmu/@(formatted_topic)/out");
  } else {
    topicName.append(name);
  }
  Wparam.topic.topicName = topicName;
	mp_publisher_ = Domain::createPublisher(
    mp_participant_,
    Wparam,
    static_cast<PublisherListener *>(&m_listener_));
  if (mp_publisher_ == nullptr) {
		throw std::runtime_error("@(topic)_Publisher::init: Failed to create publisher");
	}

  RCLCPP_INFO(node_->get_logger(), "@(topic) publisher online");
}

/**
 * @@brief Checks that a new subscription is a match.
 *
 * @@param pub Pointer to the publisher.
 * @@param info Matching information.
 */
void @(topic)_Publisher::PubListener::onPublicationMatched(Publisher * pub, MatchingInfo & info)
{
  // We support intra-process communication
  bool is_on_same_process = pub->getGuid().is_on_same_process_as(info.remoteEndpointGuid);

	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = true;
	for (size_t i = 0; i < 6; i++) {
		if (pub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint || is_on_same_process) {
		if (info.status == MATCHED_MATCHING) {
			n_matched++;
      RCLCPP_INFO(rclcpp::get_logger("RTPS Listener"), "@(topic) publisher matched");
		} else {
			n_matched--;
			RCLCPP_INFO(rclcpp::get_logger("RTPS Listener"), "@(topic) publisher unmatched");
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
  mp_publisher_->write(msg);
}

} // namespace MicroRTPSAgent
