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

#include <rclcpp/rclcpp.hpp>

namespace MicroRTPSAgent
{

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

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__TYPES_HPP_
