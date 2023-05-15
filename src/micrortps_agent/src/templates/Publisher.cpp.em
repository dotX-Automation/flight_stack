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

#include <stdexcept>

#include "@(topic)_Publisher.hpp"

#include <fastrtps/Domain.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;

namespace MicroRTPSAgent
{

/**
 * @@brief Constructor.
 */
@(topic)_Publisher::@(topic)_Publisher(rclcpp::Node * node, const std::string & ns)
: node_(node),
  ns_(ns),
  mp_participant_(nullptr),
  mp_publisher_(nullptr)
{}

/**
 * @@brief Destructor.
 */
@(topic)_Publisher::~@(topic)_Publisher()
{
  Domain::removeParticipant(mp_participant);
}

/**
 * @@brief Initializes this publisher.
 *
 * @@throws RuntimeError if an error occurs in initialization.
 */
void @(topic)_Publisher::init()
{
  // Create RTPSParticipant
	ParticipantAttributes PParam;
  PParam.domainId = 0;
  PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
  PParam.rtps.builtin.writerHistoryMemoryPolicy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

  // Set participant name
  std::string node_name = ns_;
	node_name.append("/@(topic)_publisher");
	PParam.rtps.setName(node_name.c_str());

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
  topicName.append("/fmu/@(formatted_topic)/out");
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
	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = false;

	for (size_t i = 0; i < 6; i++) {
		if (pub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint) {
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
