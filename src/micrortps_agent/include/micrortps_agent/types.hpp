/**
 * Definition of data types for the MicroRTPS ROS 2 node implementation.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
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

#ifndef MICRORTPS_AGENT__TYPES_HPP_
#define MICRORTPS_AGENT__TYPES_HPP_

namespace MicroRTPSAgent
{

/**
 * Holds an outbound message in the queue.
 */
struct OutboundMsg
{
  uint8_t topic_id;
  void * msg;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__TYPES_HPP_
