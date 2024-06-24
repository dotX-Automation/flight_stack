/**
 * Timesync protocol wrapper for the microRTPS Bridge.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 * Nuno Marques <nuno.marques@dronesolutions.io>
 * Julian Kent <julian@auterion.com>
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

#ifndef MICRORTPS_AGENT__TIMESYNC_HPP_
#define MICRORTPS_AGENT__TIMESYNC_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "Timesync_Publisher.hpp"
#include "TimesyncStatus_Publisher.hpp"

#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

using timesync_msg_t = px4_msgs::msg::Timesync;
using timesync_status_msg_t = px4_msgs::msg::TimesyncStatus;
using TimesyncPublisher = flight_stack::Timesync_Publisher;
using TimesyncStatusPublisher = flight_stack::TimesyncStatus_Publisher;

namespace flight_stack
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

class TimeSync
{
public:
  TimeSync(rclcpp::Node * node, bool debug);
  virtual ~TimeSync();

  void start(TimesyncPublisher::SharedPtr pub, TimesyncStatusPublisher::SharedPtr status_pub);
  void stop();

  void reset();

  uint64_t getROSTimeNSec() const;
  uint64_t getROSTimeUSec() const;

  bool addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns);

  void processTimesyncMsg(timesync_msg_t * msg, TimesyncPublisher::SharedPtr pub);

  timesync_msg_t newTimesyncMsg();
  timesync_status_msg_t newTimesyncStatusMsg();

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
   * @brief Updates the offset of the time sync filter
   * @param[in] offset The value of the offset to update to
   */
  inline void updateOffset(const uint64_t & offset)
  {
    _offset_ns.store(offset, std::memory_order_relaxed);
  }

  /** Timesync msg Getters **/
  inline uint64_t getMsgTimestamp(const timesync_msg_t * msg) {return msg->timestamp();}
  inline uint8_t getMsgSeq(const timesync_msg_t * msg) {return msg->seq();}
  inline int64_t getMsgTC1(const timesync_msg_t * msg) {return msg->tc1();}
  inline int64_t getMsgTS1(const timesync_msg_t * msg) {return msg->ts1();}

  /** Common timestamp setter **/
  template<typename T>
  inline void setMsgTimestamp(T * msg, const uint64_t & timestamp) {msg->timestamp() = timestamp;}

  /** Timesync msg Setters **/
  inline void setMsgSeq(timesync_msg_t * msg, const uint8_t & seq) {msg->seq() = seq;}
  inline void setMsgTC1(timesync_msg_t * msg, const int64_t & tc1) {msg->tc1() = tc1;}
  inline void setMsgTS1(timesync_msg_t * msg, const int64_t & ts1) {msg->ts1() = ts1;}

  /** Timesync Status msg Setters **/
  inline void setMsgSourceProtocol(timesync_status_msg_t * msg, const uint8_t & source_protocol)
  {
    msg->source_protocol() = source_protocol;
  }
  inline void setMsgRemoteTimeStamp(timesync_status_msg_t * msg, const uint64_t & remote_timestamp)
  {
    msg->remote_timestamp() = remote_timestamp;
  }
  inline void setMsgObservedOffset(timesync_status_msg_t * msg, const int64_t & observed_offset)
  {
    msg->observed_offset() = observed_offset;
  }
  inline void setMsgEstimatedOffset(timesync_status_msg_t * msg, const int64_t & estimated_offset)
  {
    msg->estimated_offset() = estimated_offset;
  }
  inline void setMsgRoundTripTime(timesync_status_msg_t * msg, const uint32_t & round_trip_time)
  {
    msg->round_trip_time() = round_trip_time;
  }
};

} // namespace flight_stack

#endif // MICRORTPS_AGENT__TIMESYNC_HPP_
