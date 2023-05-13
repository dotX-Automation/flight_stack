/**
 * Definition of data types for the MicroRTPS ROS 2 node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 12, 2023
 */

#ifndef MICRORTPS_AGENT__TYPES_HPP_
#define MICRORTPS_AGENT__TYPES_HPP_

#include <memory>
#include <queue>

#include <fastcdr/Cdr.h>

#include <rclcpp/rclcpp.hpp>

namespace MicroRTPSAgent
{

/* Generic message pointer type. */
typedef std::shared_ptr<void> MsgSharedPtr;

/**
 * Holds an outbound message in the queue.
 */
struct OutboundMsg
{
  uint8_t topic_id;
  MsgSharedPtr msg;
};

#ifndef BUILDING_AGENT
// Only for IDE linting purposes
#warning "Building agent with dummy RTPSTopics object"
class RTPSTopics
{
public:
  RTPSTopics(
    rclcpp::Node * node,
    std::shared_ptr<std::queue<OutboundMsg>> outbound_queue,
    std::shared_ptr<std::mutex> outbound_queue_lk,
    std::shared_ptr<std::condition_variable> outbound_queue_cv,
    bool debug = false)
  {
    (void)node;
    (void)outbound_queue;
    (void)outbound_queue_lk;
    (void)outbound_queue_cv;
    (void)debug;
  }
  ~RTPSTopics() {}

  void publish(const uint8_t topic_ID, char * data_buffer, size_t len)
  {
    (void)topic_ID;
    (void)data_buffer;
    (void)len;
  }

  bool getMsg(const uint8_t topic_ID, MsgSharedPtr msg, eprosima::fastcdr::Cdr & scdr)
  {
    (void)topic_ID;
    (void)msg;
    (void)scdr;
    return false;
  }

  void discardMsg(const uint8_t topic_ID, MsgSharedPtr msg)
  {
    (void)topic_ID;
    (void)msg;
  }

  typedef std::shared_ptr<RTPSTopics> SharedPtr;
};
#else
#include <RTPSTopics.hpp>
#endif // BUILDING_AGENT

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__TYPES_HPP_
