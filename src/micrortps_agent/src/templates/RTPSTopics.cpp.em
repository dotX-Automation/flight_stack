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
: debug_(debug),
  node_(node),
  outbound_queue_(outbound_queue),
  outbound_queue_lk_(outbound_queue_lk),
  outbound_queue_cv_(outbound_queue_cv)
{
  // Initialize subscribers
  RCLCPP_WARN(node_->get_logger(), "Initializing subscribers...");
@[for topic in recv_topics]@
  @(topic)_sub_ = std::make_shared<@(topic)_Subscriber>(
    node_,
    outbound_queue_,
    outbound_queue_lk_,
    outbound_queue_cv_,
    @(msgs[0].index(topic) + 1));
  @(topic)_sub_->init();

@[end for]@

  // Initialize publishers
  RCLCPP_WARN(node_->get_logger(), "Initializing publishers...");
@[for topic in send_topics]@
@[    if topic == 'Timesync' or topic == 'timesync']@
  timesync_pub_ = std::make_shared<@(topic)_Publisher>(node_);
  timesync_pub_->init();
  timesync_fmu_in_pub_ = std::make_shared<@(topic)_Publisher>(node_);
  timesync_fmu_in_pub_->init("/fmu/timesync/in");
@[    elif topic == 'TimesyncStatus' or topic == 'timesync_status']@
  timesync_status_pub_ = std::make_shared<@(topic)_Publisher>(node_);
  timesync_status_pub_->init();
@[    else]@
  @(topic)_pub_ = std::make_shared<@(topic)_Publisher>(node_);
  @(topic)_pub_->init();
@[    end if]@

@[end for]@
  vehicle_local_position_stamped_pub_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPositionStamped>(
    "~/fmu/vehicle_local_position_stamped/out",
    rclcpp::QoS(10));
  vehicle_attitude_stamped_pub_ = node_->create_publisher<px4_msgs::msg::VehicleAttitudeStamped>(
    "~/fmu/vehicle_attitude_stamped/out",
    rclcpp::QoS(10));
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/fmu/imu/out",
    rclcpp::QoS(10));

  // Initialize Timesync handler
  RCLCPP_WARN(node_->get_logger(), "Initializing Timesync handler...");
  _timesync = std::make_shared<TimeSync>(node_, debug_);
  _timesync->start(
    timesync_fmu_in_pub_,
    timesync_status_pub_);
}

/**
 * @@brief Destructor.
 */
RTPSTopics::~RTPSTopics()
{
  // Stop Timesync handler
  _timesync->stop();

  // Destroy subscribers
@[for topic in recv_topics]@
  @(topic)_sub_.reset();
@[end for]@
  RCLCPP_WARN(node_->get_logger(), "Subscribers terminated");

  // Destroy publishers
@[for topic in send_topics]@
@[    if topic == 'Timesync' or topic == 'timesync']@
  timesync_fmu_in_pub_.reset();
  timesync_pub_.reset();
@[    elif topic == 'TimesyncStatus' or topic == 'timesync_status']@
  timesync_status_pub_.reset();
@[    else]@
  @(topic)_pub_.reset();
@[    end if]@
@[end for]@
  vehicle_local_position_stamped_pub_.reset();
  vehicle_attitude_stamped_pub_.reset();
  imu_pub_.reset();
  RCLCPP_WARN(node_->get_logger(), "Publishers terminated");

  // Destroy Timesync handler
  _timesync.reset();

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
void RTPSTopics::sync_timestamp_of_inbound_data(T & msg) {
	uint64_t timestamp = getMsgTimestamp(&msg);
	uint64_t timestamp_sample = getMsgTimestampSample(&msg);
	_timesync->subtractOffset(timestamp);
	setMsgTimestamp(&msg, timestamp);
	_timesync->subtractOffset(timestamp_sample);
	setMsgTimestampSample(&msg, timestamp_sample);
}

/**
 * @@brief Deserializes and publishes a message to the data space.
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

      // Deserialize the message
		  @(topic)_msg_t msg;
		  eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		  eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
      try {
		    msg.deserialize(cdr_des);
      } catch (const eprosima::fastcdr::exception::BadParamException & e) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "RTPSTopics::publish(%hhu): BadParamException: %s, dropping message",
          topic_ID,
          e.what());
        return;
      } catch (const eprosima::fastcdr::exception::NotEnoughMemoryException & e) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "RTPSTopics::publish(%hhu): NotEnoughMemoryException: %s, dropping message",
          topic_ID,
          e.what());
        return;
      }

@[    if topic == 'Timesync' or topic == 'timesync']@
      // Process Timesync message
		  _timesync->processTimesyncMsg(&msg, timesync_pub_);
@[    end if]@

		  // Apply timestamp offset
		  sync_timestamp_of_inbound_data(msg);

      // Publish the message
@[    if topic == 'Timesync' or topic == 'timesync']@
      timesync_pub_->publish(&msg);
@[    elif topic == 'TimesyncStatus' or topic == 'timesync_status']@
      timesync_status_pub_->publish(&msg);
@[    elif topic == 'VehicleLocalPosition' or topic == 'vehicle_local_position']@
      @(topic)_pub_->publish(&msg);

      // Build a ROS 2 stamped message and publish it as well
      px4_msgs::msg::VehicleLocalPositionStamped pos_msg{};
      pos_msg.set__timestamp(msg.timestamp());
      pos_msg.set__timestamp_sample(msg.timestamp_sample());
      pos_msg.set__xy_valid(msg.xy_valid());
      pos_msg.set__z_valid(msg.z_valid());
      pos_msg.set__v_xy_valid(msg.v_xy_valid());
      pos_msg.set__v_z_valid(msg.v_z_valid());
      pos_msg.set__x(msg.x());
      pos_msg.set__y(msg.y());
      pos_msg.set__z(msg.z());
      pos_msg.set__vx(msg.vx());
      pos_msg.set__vy(msg.vy());
      pos_msg.set__vz(msg.vz());
      pos_msg.set__ax(msg.ax());
      pos_msg.set__ay(msg.ay());
      pos_msg.set__az(msg.az());
      pos_msg.set__eph(msg.eph());
      pos_msg.set__epv(msg.epv());
      pos_msg.set__evh(msg.evh());
      pos_msg.set__evv(msg.evv());
      pos_msg.header.set__frame_id("odom");
      pos_msg.header.stamp.set__sec(msg.timestamp() / 1000000);
      pos_msg.header.stamp.set__nanosec((msg.timestamp() % 1000000) * 1000);
      vehicle_local_position_stamped_pub_->publish(pos_msg);
@[    elif topic == 'VehicleAttitude' or topic == 'vehicle_attitude']@
      @(topic)_pub_->publish(&msg);

      // Build a ROS 2 stamped message and publish it as well
      px4_msgs::msg::VehicleAttitudeStamped att_msg{};
      att_msg.set__timestamp(msg.timestamp());
      att_msg.set__timestamp_sample(msg.timestamp_sample());
      att_msg.q[0] = msg.q()[0];
      att_msg.q[1] = msg.q()[1];
      att_msg.q[2] = msg.q()[2];
      att_msg.q[3] = msg.q()[3];
      att_msg.delta_q_reset[0] = msg.delta_q_reset()[0];
      att_msg.delta_q_reset[1] = msg.delta_q_reset()[1];
      att_msg.delta_q_reset[2] = msg.delta_q_reset()[2];
      att_msg.delta_q_reset[3] = msg.delta_q_reset()[3];
      att_msg.set__quat_reset_counter(msg.quat_reset_counter());
      att_msg.header.set__frame_id("odom");
      att_msg.header.stamp.set__sec(msg.timestamp() / 1000000);
      att_msg.header.stamp.set__nanosec((msg.timestamp() % 1000000) * 1000);
      vehicle_attitude_stamped_pub_->publish(att_msg);
@[    elif topic == 'SensorCombined' or topic == 'sensor_combined']@
      @(topic)_pub_->publish(&msg);

      // Build a ROS 2 stamped message and publish it as well
      if (msg.accelerometer_timestamp_relative() != px4_msgs::msg::SensorCombined_Constants::RELATIVE_TIMESTAMP_INVALID) {
        sensor_msgs::msg::Imu imu_msg{};
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.angular_velocity.set__x(msg.gyro_rad()[0]);
        imu_msg.angular_velocity.set__y(msg.gyro_rad()[1]);
        imu_msg.angular_velocity.set__z(msg.gyro_rad()[2]);
        imu_msg.linear_acceleration.set__x(msg.accelerometer_m_s2()[0]);
        imu_msg.linear_acceleration.set__y(msg.accelerometer_m_s2()[1]);
        imu_msg.linear_acceleration.set__z(msg.accelerometer_m_s2()[2]);
        imu_msg.header.set__frame_id("/fmu/imu_link");
        imu_msg.header.stamp.set__sec(msg.timestamp() / 1000000);
        imu_msg.header.stamp.set__nanosec((msg.timestamp() % 1000000) * 1000);
        imu_pub_->publish(imu_msg);
      }
@[    else]@
		  @(topic)_pub_->publish(&msg);
@[    end if]@
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

/**rclcpp::QoS(
 )* @@brief Synchonizes the timestamp of an outbound message.
 *
 * @@param msg Pointer to the message to synchronize.
 */
template<typename T>
void RTPSTopics::sync_timestamp_of_outbound_data(T * msg) {
	uint64_t timestamp = getMsgTimestamp(msg);
	uint64_t timestamp_sample = getMsgTimestampSample(msg);
	_timesync->addOffset(timestamp);
	setMsgTimestamp(msg, timestamp);
	_timesync->addOffset(timestamp_sample);
	setMsgTimestampSample(msg, timestamp_sample);
}

/**
 * @@brief Converts a message to a Fast-CDR buffer.
 *
 * @@param topic_ID ID of the topic from which the message came.
 * @@param msg Pointer to the message to serialize.
 * @@param scdr Reference to the Fast-CDR buffer to serialize the message into.
 *
 * @@return True if the message was successfully serialized, false otherwise.
 */
bool RTPSTopics::getMsg(const uint8_t topic_ID, void * msg, eprosima::fastcdr::Cdr & scdr)
{
	bool ret = false;

	switch (topic_ID)
  {
@[for topic in recv_topics]@
	  case @(msgs[0].index(topic) + 1):
    {
      // @(topic)

      // Cast the pointer to the correct message type
		  @(topic)_msg_t * msg_ptr = static_cast<@(topic)_msg_t *>(msg);

			// Apply timestamp offset
			sync_timestamp_of_outbound_data(msg_ptr);

      // Serialize the message into a Fast-CDR buffer
			msg_ptr->serialize(scdr);
      delete msg_ptr;

			ret = true;
		}
		break;

@[end for]@
	  default:
      RCLCPP_FATAL(
        node_->get_logger(),
        "RTPSTopics::getMsg: Unexpected topic ID (%hhu) to serialize",
        topic_ID);
	  	throw std::runtime_error("RTPSTopics::getMsg: Unexpected topic ID to serialize");
	  }

	return ret;
}

/**
 * @@brief Discards an enqueued message.
 *
 * @@param topic_ID ID of the topic from which the message came.
 * @@param msg Pointer to the message to discard.
 */
void RTPSTopics::discardMsg(const uint8_t topic_ID, void * msg)
{
  switch (topic_ID)
  {
@[for topic in recv_topics]@
	  case @(msgs[0].index(topic) + 1):
    {
      // @(topic)

      // Cast the pointer to the correct message type and destroy the object
		  @(topic)_msg_t * msg_ptr = static_cast<@(topic)_msg_t *>(msg);
      delete msg_ptr;
		}
		break;

@[end for]@
	  default:
      RCLCPP_FATAL(
        node_->get_logger(),
        "RTPSTopics::discardMsg: Unexpected topic ID (%hhu) to discard",
        topic_ID);
	  	throw std::runtime_error("RTPSTopics::discardMsg: Unexpected topic ID");
	  }
}

} // namespace MicroRTPSAgent
