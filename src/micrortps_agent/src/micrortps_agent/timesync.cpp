/**
 * Timesync protocol wrapper implementation.
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

#include <cmath>

#include "timesync.hpp"

namespace micrortps_agent
{

/**
 * @brief Creates a new TimeSync object.
 *
 * @param debug If true, prints debug messages.
 */
TimeSync::TimeSync(rclcpp::Node * node, bool debug)
	: _offset_ns(-1),
    _node(node),
	  _skew_ns_per_sync(0.0),
	  _num_samples(0),
	  _request_reset_counter(0),
	  _last_msg_seq(0),
	  _last_remote_msg_seq(0),
	  _debug(debug)
{}

/**
 * @brief Destroys a Timesync object.
 */
TimeSync::~TimeSync()
{
  stop();
}

/**
 * @brief Starts the timesync worker thread.
 */
void TimeSync::start(TimesyncPublisher::SharedPtr pub, TimesyncStatusPublisher::SharedPtr status_pub)
{
  // Clear previous instances
	stop();
  _request_stop.store(false, std::memory_order_release);

	auto run_timesync = [this, pub]() -> void {
		while (!_request_stop.load(std::memory_order_acquire)) {
			timesync_msg_t msg = newTimesyncMsg();

			pub->publish(&msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};
  auto run_timesync_status = [this, status_pub]() -> void {
		while (!_request_stop.load(std::memory_order_acquire)) {
			timesync_status_msg_t status_msg = newTimesyncStatusMsg();

			status_pub->publish(&status_msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};

	_send_timesync_thread.reset(new std::thread(run_timesync));
  _send_timesync_status_thread.reset(new std::thread(run_timesync_status));
}

/**
 * @brief Stops worker threads.
 */
void TimeSync::stop()
{
	_request_stop.store(true, std::memory_order_release);

	if (_send_timesync_thread && _send_timesync_thread->joinable()) {
    _send_timesync_thread->join();
    RCLCPP_INFO(_node->get_logger(), "TimeSync::stop: _send_timesync_thread joined");
  }
	if (_send_timesync_status_thread && _send_timesync_status_thread->joinable()) {
    _send_timesync_status_thread->join();
    RCLCPP_INFO(_node->get_logger(), "TimeSync::stop: _send_timesync_status_thread joined");
  }
}

/**
 * @brief Resets the filter.
 */
void TimeSync::reset()
{
	_num_samples = 0;
	_request_reset_counter = 0;
}

/**
 * ROS time will match the system time, which corresponds to the system-wide real time since epoch.
 * If the use_sim_time parameter is set for the node to which this object is attached,
 * the simulation time is grabbed by the node and used instead.
 * More info about ROS 2 clock and time in: https://design.ros2.org/articles/clock_and_time.html
 */

/**
 * @brief Get ROS time in nanoseconds.
 *
 * @return ROS time in nanoseconds.
 */
uint64_t TimeSync::getROSTimeNSec() const
{
	return _node->now().nanoseconds();
}

/**
 * @brief Get ROS time in microseconds.
 *
 * @return ROS time in microseconds.
 */
uint64_t TimeSync::getROSTimeUSec() const
{
	return RCL_NS_TO_US(getROSTimeNSec());
}

/**
 * @brief Adds a time offset measurement to be filtered.
 *
 * @param local_t1_ns The CLOCK_MONOTONIC time in nanoseconds when the message was sent.
 * @param remote_t2_ns The remote CLOCK_MONOTONIC time in nanoseconds.
 * @param local_t3_ns The current CLOCK_MONOTONIC time in nanoseconds.
 *
 * @return true or false depending if the time offset was updated.
 */
bool TimeSync::addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns)
{
	_rtt.store(local_t3_ns - local_t1_ns, std::memory_order_release);
	_remote_time_stamp.store(remote_t2_ns, std::memory_order_release);

	// Assume RTT is evenly split both directions
	int64_t remote_t3_ns = remote_t2_ns + _rtt.load(std::memory_order_acquire) / 2ll;

	int64_t measurement_offset = remote_t3_ns - local_t3_ns;

	if (_request_reset_counter > REQUEST_RESET_COUNTER_THRESHOLD) {
		reset();

		if (_debug) {
      RCLCPP_INFO(
        _node->get_logger(),
        "Timesync clock changed, filter reset");
    }
	}

	if (_num_samples == 0) {
		updateOffset(measurement_offset);
		_skew_ns_per_sync = 0;
	}

	if (_num_samples >= WINDOW_SIZE) {
		if (std::abs(measurement_offset - _offset_ns.load(std::memory_order_acquire)) > TRIGGER_RESET_THRESHOLD_NS) {
			_request_reset_counter++;

			if (_debug) {
        RCLCPP_INFO(
          _node->get_logger(),
          "Timesync offset outlier, discarding");
      }

			return false;
		} else {
			_request_reset_counter = 0;
		}
	}

	// Ignore if RTT > 50ms
	if (_rtt.load(std::memory_order_acquire) > 50ll * 1000ll * 1000ll) {
		if (_debug) {
      RCLCPP_WARN(
        _node->get_logger(),
        "RTT too high for timesync: %lld ms",
        _rtt.load(std::memory_order_acquire) / (1000ll * 1000ll));
    }

		return false;
	}

	double alpha = ALPHA_FINAL;
	double beta = BETA_FINAL;

	if (_num_samples < WINDOW_SIZE) {
		double schedule = (double)_num_samples / WINDOW_SIZE;
		double s = 1. - exp(.5 * (1. - 1. / (1. - schedule)));
		alpha = (1. - s) * ALPHA_INITIAL + s * ALPHA_FINAL;
		beta = (1. - s) * BETA_INITIAL + s * BETA_FINAL;
	}

	_offset_prev.store(_offset_ns.load(std::memory_order_acquire), std::memory_order_release);
	updateOffset(static_cast<int64_t>(
    (_skew_ns_per_sync + _offset_ns.load(std::memory_order_acquire)) * (1. - alpha)
    + measurement_offset * alpha));
	_skew_ns_per_sync =
		static_cast<int64_t>(
      beta * (_offset_ns.load(std::memory_order_acquire) - _offset_prev.load(std::memory_order_acquire))
      + (1. - beta) * _skew_ns_per_sync);

	_num_samples++;

	return true;
}

/**
 * @brief Processes an incoming timesync message.
 *
 * @param msg Message to be processed.
 */
void TimeSync::processTimesyncMsg(timesync_msg_t * msg, TimesyncPublisher::SharedPtr pub)
{
	if (getMsgSeq(msg) != _last_remote_msg_seq) {
		_last_remote_msg_seq = getMsgSeq(msg);

		if (getMsgTC1(msg) > 0) {
			if (!addMeasurement(getMsgTS1(msg), getMsgTC1(msg), getROSTimeNSec())) {
				if (_debug) {
          RCLCPP_WARN(
            _node->get_logger(),
            "Offset not updated");
        }
			}

		} else if (getMsgTC1(msg) == 0) {
			setMsgTimestamp(msg, getROSTimeUSec());
			setMsgSeq(msg, getMsgSeq(msg) + 1);
			setMsgTC1(msg, getROSTimeNSec());

			pub->publish(msg);
		}
	}
}

/**
 * @brief Creates a new timesync message to be sent to the remote entity.
 *
 * @return A new timesync message with local origin and timestamp.
 */
timesync_msg_t TimeSync::newTimesyncMsg()
{
	timesync_msg_t msg{};

	setMsgTimestamp(&msg, getROSTimeUSec());
	setMsgSeq(&msg, _last_msg_seq);
	setMsgTC1(&msg, 0);
	setMsgTS1(&msg, getROSTimeNSec());

	_last_msg_seq++;

	return msg;
}

/**
 * @brief Creates a new timesync status message to be sent.
 *
 * @return A new timesync status message with local origin and timestamp.
 */
timesync_status_msg_t TimeSync::newTimesyncStatusMsg()
{
	timesync_status_msg_t msg = {};

	setMsgTimestamp(&msg, getROSTimeUSec());
	setMsgSourceProtocol(&msg, 1); // SOURCE_PROTOCOL_RTPS
	setMsgRemoteTimeStamp(&msg, _remote_time_stamp.load(std::memory_order_acquire) / 1000ULL);
	setMsgObservedOffset(&msg, _offset_prev.load(std::memory_order_acquire));
	setMsgEstimatedOffset(&msg, _offset_ns.load(std::memory_order_acquire));
	setMsgRoundTripTime(&msg, _rtt.load(std::memory_order_acquire) / 1000ll);

	return msg;
}

} // namespace micrortps_agent
