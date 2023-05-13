@# EmPy template for generating RTPSTopics.cpp file.
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
 * DDS topics wrapper object implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 13, 2023
 */

#include "RTPSTopics.hpp"

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
RTPSTopics::RTPSTopics(
  rclcpp::Node * node,
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
  std::shared_ptr<std::mutex> outbound_queue_lk,
  std::shared_ptr<std::condition_variable> outbound_queue_cv,
  bool debug)
: node_(node),
  outbound_queue_(outbound_queue),
  outbound_queue_lk_(outbound_queue_lk),
  outbound_queue_cv_(outbound_queue_cv),
  debug_(debug)
{
  // Initialize subscribers
  RCLCPP_WARN(node_->get_logger(), "Initializing subscribers...");
@[for topic in recv_topics]@
  @(topic)_sub_ = std::make_shared<@(topic)_Subscriber>(
    node_,
    outbound_queue_,
    outbound_queue_lk_,
    outbound_queue_cv_);
  @(topic)_sub_->init();
@[end for]@

  // Initialize publishers
  RCLCPP_WARN(node_->get_logger(), "Initializing publishers...");
@[for topic in send_topics]@
  @(topic)_pub_ = std::make_shared<@(topic)_Publisher>(node_);
  @(topic)_pub_->init();
@[end for]@

  // Initialize TimeSync object
  timesync_fmu_in_ros2_pub_ = node_->create_publisher<px4_msgs::msg::Timesync>(
    "~/fmu/timesync/in",
    rclcpp::QoS(10));
  timesync_status_ros2_pub_ = TimesyncStatus_pub_->get_publisher();
  timesync_ = std::make_shared<TimeSync>(
    node_,
    timesync_fmu_in_ros2_pub_,
    timesync_status_ros2_pub_,
    debug_);
  timesync_->start();
  RCLCPP_WARN(node_->get_logger(), "Timesync handler started");
}

/**
 * @@brief Destructor.
 */
RTPSTopics::~RTPSTopics()
{
  // Destroy the TimeSync object
  timesync_.reset();
  timesync_fmu_in_ros2_pub_.reset();
  RCLCPP_WARN(node_->get_logger(), "Timesync handler terminated");

  // Destroy subscribers
@[for topic in recv_topics]@
  @(topic)_sub_.reset();
@[end for]@
  RCLCPP_WARN(node_->get_logger(), "Subscribers terminated");

  // Destroy publishers
@[for topic in send_topics]@
  @(topic)_pub_.reset();
@[end for]@
  RCLCPP_WARN(node_->get_logger(), "Publishers terminated");

  // Clear the outbound message queue
  {
    std::unique_lock<std::mutex> lk(*outbound_queue_lk_);
    while (!outbound_queue_->empty()) {
      OutboundMsg msg_struct = outbound_queue_->front();
      discardMsg(msg_struct.topic_id, msg_struct.msg);
      outbound_queue_->pop();
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Outbound message queue cleared");
}

/**
 * @@brief Synchronizes the timestamp of an inbound message.
 *
 * @@param msg Pointer to the message to synchronize.
 */
template<typename T>
void RTPSTopics::sync_timestamp_of_inbound_data(std::shared_ptr<T> msg) {
	uint64_t timestamp = getMsgTimestamp(msg);
	uint64_t timestamp_sample = getMsgTimestampSample(msg);
	timesync_->subtractOffset(timestamp);
	setMsgTimestamp(msg, timestamp);
	timesync_->subtractOffset(timestamp_sample);
	setMsgTimestampSample(msg, timestamp_sample);
}

/**
 * @@brief Deserializes and publishes a message to the ROS 2 data space.
 *
 * @@param topic_ID ID of the topic to publish the message to.
 * @@param data_buffer Pointer to the buffer containing the serialized message.
 * @@param len Length of the serialized message.
 */
void RTPSTopics::publish(const uint8_t topic_ID, char * data_buffer, size_t len)
{
	switch (topic_ID)
  {
@[for topic in send_topics]@
	case @(msgs[0].index(topic) + 1):
    {
      // @(topic)

      // Deserialize the message into a ROS 2 message data structure,
      // using the Fast-CDR library definition of messages that ROS 2 uses
      // TODO Handle exceptions
		  @(topic)_msg_t::SharedPtr msg = std::make_shared<@(topic)_msg_t>();
		  eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		  eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		  msg->deserialize(cdr_des);

@[    if topic == 'Timesync' or topic == 'timesync']@
      // Process Timesync message
		  timesync_->processTimesyncMsg(msg);
@[    end if]@

		  // Apply timestamp offset
		  sync_timestamp_of_inbound_data(msg);

      // Publish the ROS 2 message
		  @(topic)_pub_->publish(*msg);
	  }
	  break;

@[end for]@
	default:
    RCLCPP_INFO(
      node_->get_logger(),
      "RTPSTopics::publish: Unexpected topic ID (%hhu) to publish",
      topic_ID);
		break;
	}
}

/**
 * @@brief Synchonizes the timestamp of an outbound message.
 *
 * @@param msg Pointer to the message to synchronize.
 */
template<typename T>
void RTPSTopics::sync_timestamp_of_outbound_data(std::shared_ptr<T> msg) {
	uint64_t timestamp = getMsgTimestamp(msg);
	uint64_t timestamp_sample = getMsgTimestampSample(msg);
	timesync_->addOffset(timestamp);
	setMsgTimestamp(msg, timestamp);
	timesync_->addOffset(timestamp_sample);
	setMsgTimestampSample(msg, timestamp_sample);
}

/**
 * @@brief Converts a ROS 2 message to a Fast-CDR buffer.
 *
 * @@param topic_ID ID of the topic from which the message came.
 * @@param msg Pointer to the ROS 2 message to serialize.
 * @@param scdr Reference to the Fast-CDR buffer to serialize the message into.
 *
 * @@return True if the message was successfully serialized, false otherwise.
 */
bool RTPSTopics::getMsg(const uint8_t topic_ID, MsgSharedPtr msg, eprosima::fastcdr::Cdr & scdr)
{
	bool ret = false;

	switch (topic_ID)
  {
@[for topic in recv_topics]@
	  case @(msgs[0].index(topic) + 1):
    {
      // @(topic)

      // Cast the shared pointer to the correct message type
		  @(topic)_msg_t::SharedPtr msg = std::static_pointer_cast<@(topic)_msg_t>(msg);

			// Apply timestamp offset
			sync_timestamp_of_outbound_data(msg);

      // Serialize the message into a Fast-CDR buffer
			msg->serialize(scdr);

			ret = true;
		}
		break;

@[end for]@
	  default:
      RCLCPP_INFO(
        node_->get_logger(),
        "RTPSTopics::getMsg: Unexpected topic ID (%hhu) to serialize",
        topic_ID);
	  	break;
	  }

	return ret;
}

/**
 * @@brief Discards an enqueued ROS 2 message.
 *
 * @@param topic_ID ID of the topic from which the message came.
 * @@param msg Pointer to the ROS 2 message to discard.
 */
void RTPSTopics::discardMsg(const uint8_t topic_ID, MsgSharedPtr msg)
{
  switch (topic_ID)
  {
@[for topic in recv_topics]@
	  case @(msgs[0].index(topic) + 1):
    {
      // @(topic)

      // Cast the shared pointer to the correct message type and destroy the object
		  @(topic)_msg_t::SharedPtr msg = std::static_pointer_cast<@(topic)_msg_t>(msg);
      msg.reset();
		}
		break;

@[end for]@
	  default:
      RCLCPP_INFO(
        node_->get_logger(),
        "RTPSTopics::discardMsg: Unexpected topic ID (%hhu) to discard",
        topic_ID);
	  	break;
	  }
}

} // namespace MicroRTPSAgent
