/**
 * MicroRTPS ROS 2 node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 9, 2023
 */

#ifndef MICRORTPS_AGENT__MICRORTPS_AGENT_HPP_
#define MICRORTPS_AGENT__MICRORTPS_AGENT_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <pthread.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>

#include <micrortps_agent/types.hpp>
#include <transport/transport.hpp>

namespace MicroRTPSAgent
{

/**
 * Handles DDS-level communications with the PX4 FMU.
 */
class AgentNode : public DUANode::NodeBase
{
public:
  AgentNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  ~AgentNode();

private:
  /* Node parameters declaration routine. */
  void init_parameters();

  /* Node parameters and validation routines. */
  std::string transport_type_;
  bool validate_transport_type(const rclcpp::Parameter & p);

  /* Outbound messages queue. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::shared_ptr<std::mutex> outbound_queue_lk_;
  std::shared_ptr<std::condition_variable> outbound_queue_cv_;

  /* Transport handler. */
  Transporter::SharedPtr transporter_;
  void init_transporter();

  /* DDS topics handler (and publisher/subscriber objects container). */
  RTPSTopics::SharedPtr rtps_topics_;

  /* Outbound messages handler thread data. */
  std::thread sender_;
  void sender_routine();

  /* Inbound messages handler thread data. */
  std::thread receiver_;
  void receiver_routine();

  /* Internal state variables. */
  std::atomic_bool running_ = false;
  uint64_t total_sent_ = 0;
  uint64_t sent_ = 0;
  uint64_t total_read_ = 0;
  uint64_t received_ = 0;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__MICRORTPS_AGENT_HPP_
