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
#include <memory>
#include <thread>

#include <pthread.h>

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>

#include <transport/transport.hpp>

#ifndef BUILDING_AGENT
// Only for IDE linting purposes
#warning "Building agent with dummy RTPSTopics object"
struct RTPSTopics
{
  RTPSTopics(rclcpp::Node * node)
  {
    (void)node;
  }

  typedef std::shared_ptr<RTPSTopics> SharedPtr;
};
#else
#include <micrortps_agent/RTPSTopics.hpp>
#endif

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
  /* Node parameters and declaration routine. */
  void init_parameters();

  /* Node parameters validation routines. */
  bool validate_transport_type(const rclcpp::Parameter & p);

  /* DDS topics handler (and publisher/subscriber objects container). */
  RTPSTopics::SharedPtr rtps_topics_;

  /* Transport handler. */
  Transporter::SharedPtr transporter_;

  /* Sender thread data. */
  std::thread sender_;
  void sender_routine();

  /* Receiver thread data. */
  std::thread receiver_;
  void receiver_routine();

  /* Internal state variables. */
  std::atomic_bool running_ = false;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__MICRORTPS_AGENT_HPP_
