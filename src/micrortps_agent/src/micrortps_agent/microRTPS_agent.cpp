/**
 * MicroRTPS ROS 2 node implementation.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
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

#include <chrono>

#include <signal.h>

#include <micrortps_agent/microRTPS_agent.hpp>

namespace micrortps_agent
{

/**
 * @brief Constructor.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError if thread initialization fails.
 */
AgentNode::AgentNode(const rclcpp::NodeOptions & opts)
: NodeBase("microRTPS_agent", opts, true)
{
  // Initialize node parameters
  init_parameters();

  // Initialize the outbound message queue
  outbound_queue_ = std::make_shared<std::queue<OutboundMsg>>();
  outbound_queue_lk_ = std::make_shared<std::mutex>();
  outbound_queue_cv_ = std::make_shared<std::condition_variable>();

  // Initialize the transport handler
  init_transporter();

  // Initialize the DDS topics handler
  rtps_topics_ = std::make_shared<RTPSTopics>(
    this,
    outbound_queue_,
    outbound_queue_lk_,
    outbound_queue_cv_,
    this->get_parameter("link_namespace").as_string(),
    imu_variance_,
    this->get_parameter("debug").as_bool(),
    this->get_parameter("localhost_only").as_bool());

  running_.store(true, std::memory_order_release);

  // Initialize sender thread and set its CPU affinity and name
  sender_ = std::thread(&AgentNode::sender_routine, this);
  cpu_set_t sender_cpuset;
  CPU_ZERO(&sender_cpuset);
  CPU_SET(this->get_parameter("sender_cpu").as_int(), &sender_cpuset);
  if (pthread_setaffinity_np(sender_.native_handle(), sizeof(cpu_set_t), &sender_cpuset) ||
    pthread_setname_np(sender_.native_handle(), "uRTPS::sender"))
  {
    char err_msg_buf[100] = {};
    char * err_msg = strerror_r(errno, err_msg_buf, 100);
    throw std::runtime_error(
            "AgentNode::AgentNode: Failed to configure sender thread: " + std::string(err_msg));
  }

  // Initialize receiver thread and set its CPU affinity and name
  receiver_ = std::thread(&AgentNode::receiver_routine, this);
  cpu_set_t receiver_cpuset;
  CPU_ZERO(&receiver_cpuset);
  CPU_SET(this->get_parameter("receiver_cpu").as_int(), &receiver_cpuset);
  if (pthread_setaffinity_np(receiver_.native_handle(), sizeof(cpu_set_t), &receiver_cpuset) ||
    pthread_setname_np(receiver_.native_handle(), "uRTPS::receiver"))
  {
    char err_msg_buf[100] = {};
    char * err_msg = strerror_r(errno, err_msg_buf, 100);
    throw std::runtime_error(
            "AgentNode::AgentNode: Failed to configure receiver thread: " + std::string(err_msg));
  }

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Destructor.
 */
AgentNode::~AgentNode()
{
  // Stop worker threads: close transport link to wake up receiver, wake up the sender
  {
    std::lock_guard<std::mutex> lk(*outbound_queue_lk_);
    running_.store(false, std::memory_order_release);
  }
  outbound_queue_cv_->notify_one();
  transporter_->close();

  // We must make sure that the threads terminate before destroying the transporter
  // This implies sending them specifically signals that will interrupt each and every blocking
  // call they are doing
  pthread_kill(receiver_.native_handle(), SIGUSR1);
  pthread_kill(sender_.native_handle(), SIGUSR1);
  sender_.join();
  receiver_.join();
  transporter_.reset();

  // Destroy the DDS topics handler, closing ROS 2 communications and freeing memory
  rtps_topics_.reset();
}

/**
 * @brief Handles outbound messages.
 */
void AgentNode::sender_routine()
{
  char data_buffer[BUFFER_SIZE] = {};
  int64_t length = 0;

  while (true) {
    // Wait for a message to be available
    std::unique_lock<std::mutex> lk(*outbound_queue_lk_);
    outbound_queue_cv_->wait(
      lk,
      [this] {return !outbound_queue_->empty() || !running_.load(std::memory_order_acquire);});
    if (!running_.load(std::memory_order_acquire)) {
      lk.unlock();
      break;
    }
    OutboundMsg new_msg = outbound_queue_->front();
    outbound_queue_->pop();
    lk.unlock();

    // Serialize the message and send it
    size_t header_len = transporter_->get_header_length();
    eprosima::fastcdr::FastBuffer cdrbuffer(
      &data_buffer[header_len],
      sizeof(data_buffer) - header_len);
    eprosima::fastcdr::Cdr scdr(cdrbuffer);
    if (rtps_topics_->getMsg(new_msg.topic_id, new_msg.msg, scdr)) {
      length = scdr.getSerializedDataLength();

      if (0 < (length = transporter_->write(new_msg.topic_id, data_buffer, length))) {
        ++sent_;
        total_sent_ += uint64_t(length);
      }
    }
  }

  // Print statistics
  RCLCPP_INFO(
    this->get_logger(),
    "SENT: %lu msgs - %lu bytes",
    sent_,
    total_sent_);
}

/**
 * @brief Handles inbound messages.
 */
void AgentNode::receiver_routine()
{
  char data_buffer[BUFFER_SIZE] = {};
  int64_t length = 0;
  uint8_t topic_id = 255;
  std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
  std::chrono::time_point<std::chrono::steady_clock> end;

  while (running_.load(std::memory_order_acquire)) {
    // Wait for a message to be available from the transport layer, then publish it
    if (0 < (length = transporter_->read(&topic_id, data_buffer, BUFFER_SIZE))) {
      rtps_topics_->publish(topic_id, data_buffer, BUFFER_SIZE);
      ++received_;
      total_read_ += uint64_t(length);
      end = std::chrono::steady_clock::now();
    }
  }

  // Print statistics
  std::chrono::duration<double> elapsed_secs = end - start;
  RCLCPP_INFO(
    this->get_logger(),
    "RECEIVED: %lu msgs - %lu bytes - %.02f KB/s",
    received_,
    total_read_,
    static_cast<double>(total_read_) / (1000.0 * elapsed_secs.count()));
}

} // namespace micrortps_agent

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(micrortps_agent::AgentNode)
