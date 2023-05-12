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

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>

#include <micrortps_agent/types.hpp>
#include <transport/transport.hpp>
#include <timesync/timesync.hpp>

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

  /* Node parameters validation routines. */
  bool validate_transport_type(const rclcpp::Parameter & p);

  /* Outbound messages queue. */
  std::shared_ptr<std::queue<OutboundMsg>> outbound_queue_;
  std::mutex outbound_queue_lk_;
  std::condition_variable outbound_queue_cv_;

  /* Transport handler. */
  // TODO: note that when this is instantiated, inbound messages will come in, but the receiver must parse them
  Transporter::SharedPtr transporter_;

  /* DDS topics handler (and publisher/subscriber objects container). */
  // TODO: note that when this is instantiated, outbound messages will be received, but they must be processed by the sender thread
  RTPSTopics::SharedPtr rtps_topics_;

  /* Time synchronization handler. */
  TimeSync::SharedPtr timesync_;

  /* Outbound messages handler thread data. */
  std::thread sender_;
  void sender_routine();

  /* Inbound messages handler thread data. */
  std::thread receiver_;
  void receiver_routine();

  /* Internal state variables. */
  std::atomic_bool running_ = false;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__MICRORTPS_AGENT_HPP_
