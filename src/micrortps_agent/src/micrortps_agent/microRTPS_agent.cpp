/**
 * MicroRTPS ROS 2 node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 9, 2023
 */

#include <micrortps_agent/microRTPS_agent.hpp>

namespace MicroRTPSAgent
{

/**
 * @brief Destructor.
 */
AgentNode::~AgentNode()
{
  // TODO
  // Empty the outbound message queue
  {
    std::unique_lock<std::mutex> lk(*outbound_queue_lk_);
    while (!outbound_queue_->empty())
    {
      OutboundMsg msg_struct = outbound_queue_->front();
      discardMsg(msg_struct.topic_id, msg_struct.msg);
      outbound_queue_->pop();
    }
  }
}

} // namespace MicroRTPSAgent
