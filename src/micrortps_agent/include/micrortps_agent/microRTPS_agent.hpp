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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>

#include <micrortps_agent/types.hpp>
#include <transport/transport.hpp>

#ifndef BUILDING_AGENT
// Only for IDE linting purposes
#warning "Building agent with dummy RTPSTopics object"
namespace MicroRTPSAgent
{
class RTPSTopics
{
public:
  RTPSTopics(
    rclcpp::Node * node,
    std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
    std::shared_ptr<std::mutex> outbound_queue_lk,
    std::shared_ptr<std::condition_variable> outbound_queue_cv,
    std::string link_namespace,
    std::shared_ptr<std::array<double, 6>> imu_variance,
    bool debug = false,
    bool localhost_only = false)
  {
    (void)node;
    (void)outbound_queue;
    (void)outbound_queue_lk;
    (void)outbound_queue_cv;
    (void)link_namespace;
    (void)imu_variance;
    (void)debug;
    (void)localhost_only;
  }
  ~RTPSTopics() {}

  void publish(
    const uint8_t topic_ID,
    char * data_buffer,
    size_t len)
  {
    (void)topic_ID;
    (void)data_buffer;
    (void)len;
  }

  bool getMsg(const uint8_t topic_ID, void * msg, eprosima::fastcdr::Cdr & scdr)
  {
    (void)topic_ID;
    (void)msg;
    (void)scdr;
    return false;
  }

  void discardMsg(const uint8_t topic_ID, void * msg)
  {
    (void)topic_ID;
    (void)msg;
  }

  typedef std::shared_ptr<RTPSTopics> SharedPtr;
};
} // namespace MicroRTPSAgent
#else
#include <RTPSTopics.hpp>
#endif // BUILDING_AGENT

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
  std::shared_ptr<std::array<double, 6>> imu_variance_ = std::make_shared<std::array<double, 6>>();
  bool validate_transport_type(const rclcpp::Parameter & p);
  bool validate_imu_variance(const rclcpp::Parameter & p);

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
