/**
 * Timesync protocol wrapper for the microRTPS Bridge.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 * Nuno Marques <nuno.marques@dronesolutions.io>
 * Julian Kent <julian@auterion.com>
 *
 * April 3, 2023
 */

/****************************************************************************
 *
 * Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#ifndef MICRORTPS_AGENT__TIMESYNC_HPP
#define MICRORTPS_AGENT__TIMESYNC_HPP

#include "visibility_control.h"

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <rcl/time.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

namespace MicroRTPSAgent
{

/**
 * RTT estimator parameters.
 */
static constexpr double ALPHA_INITIAL = 0.05;
static constexpr double ALPHA_FINAL = 0.003;
static constexpr double BETA_INITIAL = 0.05;
static constexpr double BETA_FINAL = 0.003;
static constexpr int WINDOW_SIZE = 500;
static constexpr int64_t UNKNOWN = 0;
static constexpr int64_t TRIGGER_RESET_THRESHOLD_NS = 100ll * 1000ll * 1000ll;
static constexpr int REQUEST_RESET_COUNTER_THRESHOLD = 5;

class TIMESYNC_PUBLIC TimeSync
{
public:
  TimeSync(rclcpp::Node * node, bool debug);
  virtual ~TimeSync();

  void start();
  void stop();

  void reset();

  uint64_t getROSTimeNSec() const;
  uint64_t getROSTimeUSec() const;

  bool addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns);

  void processTimesyncMsg(px4_msgs::msg::Timesync::SharedPtr msg);

  px4_msgs::msg::Timesync::SharedPtr newTimesyncMsg();
  px4_msgs::msg::TimesyncStatus::SharedPtr newTimesyncStatusMsg();

  /**
   * @brief Get the time sync offset in nanoseconds.
   *
   * @return The offset in nanoseconds.
   */
  inline int64_t getOffset() {return _offset_ns.load(std::memory_order_acquire);}

  /**
   * @brief Sums the time sync offset to the timestamp.
   *
   * @param timestamp The timestamp to add the offset to.
   */
  inline void addOffset(uint64_t & timestamp)
  {
    timestamp = (timestamp * 1000LL + _offset_ns.load(std::memory_order_acquire)) / 1000ULL;
  }

  /**
   * @brief Substracts the time sync offset to the timestamp.
   *
   * @param timestamp The timestamp to subtract the offset from.
   */
  inline void subtractOffset(uint64_t & timestamp)
  {
    timestamp = (timestamp * 1000LL - _offset_ns.load(std::memory_order_acquire)) / 1000ULL;
  }

  typedef std::shared_ptr<TimeSync> SharedPtr;

private:
  std::atomic<int64_t> _offset_ns;
  std::atomic<int64_t> _offset_prev;
  std::atomic<uint64_t> _remote_time_stamp;
  std::atomic<uint32_t> _rtt;

  rclcpp::Node * _node;

  rclcpp::Publisher<px4_msgs::msg::Timesync>::SharedPtr _timesync_pub;
  rclcpp::Publisher<px4_msgs::msg::TimesyncStatus>::SharedPtr _status_pub;

  int64_t _skew_ns_per_sync;
  int64_t _num_samples;

  int32_t _request_reset_counter;
  uint8_t _last_msg_seq;
  uint8_t _last_remote_msg_seq;

  bool _debug;

  std::unique_ptr<std::thread> _send_timesync_thread;
  std::unique_ptr<std::thread> _send_timesync_status_thread;

  std::atomic<bool> _request_stop{false};

  /**
   * @brief Updates the offset of the time sync filter.
   *
   * @param offset The new value of the offset.
   */
  inline void TIMESYNC_LOCAL updateOffset(const uint64_t & offset)
  {
    _offset_ns.store(offset, std::memory_order_release);
  }

  /* Timesync message getters. */
  inline uint64_t TIMESYNC_LOCAL getMsgTimestamp(const px4_msgs::msg::Timesync::SharedPtr msg)
  {
    return msg->timestamp;
  }
  inline uint8_t TIMESYNC_LOCAL getMsgSeq(const px4_msgs::msg::Timesync::SharedPtr msg)
  {
    return msg->seq;
  }
  inline int64_t TIMESYNC_LOCAL getMsgTC1(const px4_msgs::msg::Timesync::SharedPtr msg)
  {
    return msg->tc1;
  }
  inline int64_t TIMESYNC_LOCAL getMsgTS1(const px4_msgs::msg::Timesync::SharedPtr msg)
  {
    return msg->ts1;
  }

  /* Timesync message setters. */
  inline void TIMESYNC_LOCAL setMsgTimestamp(px4_msgs::msg::Timesync::SharedPtr msg,
    const uint64_t & timestamp)
  {
    msg->set__timestamp(timestamp);
  }
  inline void TIMESYNC_LOCAL setMsgSeq(px4_msgs::msg::Timesync::SharedPtr msg, const uint8_t & seq)
  {
    msg->set__seq(seq);
  }
  inline void TIMESYNC_LOCAL setMsgTC1(px4_msgs::msg::Timesync::SharedPtr msg, const int64_t & tc1)
  {
    msg->set__tc1(tc1);
  }
  inline void TIMESYNC_LOCAL setMsgTS1(px4_msgs::msg::Timesync::SharedPtr msg, const int64_t & ts1)
  {
    msg->set__ts1(ts1);
  }

  /* TimesyncStatus message setters. */
  inline void TIMESYNC_LOCAL setMsgTimestamp(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const uint64_t & timestamp)
  {
    msg->set__timestamp(timestamp);
  }
  inline void TIMESYNC_LOCAL setMsgSourceProtocol(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const uint8_t & source_protocol)
  {
    msg->set__source_protocol(source_protocol);
  }
  inline void TIMESYNC_LOCAL setMsgRemoteTimeStamp(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const uint64_t & remote_timestamp)
  {
    msg->set__remote_timestamp(remote_timestamp);
  }
  inline void TIMESYNC_LOCAL setMsgObservedOffset(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const int64_t & observed_offset)
  {
    msg->set__observed_offset(observed_offset);
  }
  inline void TIMESYNC_LOCAL setMsgEstimatedOffset(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const int64_t & estimated_offset)
  {
    msg->set__estimated_offset(estimated_offset);
  }
  inline void TIMESYNC_LOCAL setMsgRoundTripTime(
    px4_msgs::msg::TimesyncStatus::SharedPtr msg,
    const uint32_t & round_trip_time)
  {
    msg->set__round_trip_time(round_trip_time);
  }
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__TIMESYNC_HPP
