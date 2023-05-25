/**
 * Transporters definitions.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 2, 2023
 */

/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#ifndef MICRORTPS_AGENT__TRANSPORT_HPP_
#define MICRORTPS_AGENT__TRANSPORT_HPP_

#include <cstring>

#include <arpa/inet.h>
#include <limits.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#define BUFFER_SIZE 1024

namespace MicroRTPSAgent
{

/**
 * Identifies the system that is using the transport.
 */
enum class System
{
  FMU,
  MISSION_COMPUTER
};

/**
 * RTPS transport module base class.
 */
class Transporter
{
public:
  Transporter(const uint8_t sys_id, const bool debug);
  virtual ~Transporter();

  virtual int init() {return 0;}
  virtual uint8_t close() {return 0;}

  ssize_t read(uint8_t * topic_id, char out_buffer[], size_t buffer_len);
  ssize_t write(const uint8_t topic_id, char buffer[], size_t length);

  constexpr size_t get_header_length() {return sizeof(struct Header);}

  typedef std::shared_ptr<Transporter> SharedPtr;

private:
  struct __attribute__((packed)) Header
  {
    char marker[3];
    uint8_t topic_id;
    uint8_t sys_id;
    uint8_t seq;
    uint8_t payload_len_h;
    uint8_t payload_len_l;
    uint8_t crc_h;
    uint8_t crc_l;
  };

protected:
  virtual ssize_t _read(void * buffer, size_t len) = 0;
  virtual ssize_t _write(void * buffer, size_t len) = 0;

  virtual bool fds_OK() = 0;

  uint16_t crc16_byte(uint16_t crc, const uint8_t data);
  uint16_t crc16(uint8_t const * buffer, size_t len);

  uint32_t _rx_buff_pos;
  char _rx_buffer[BUFFER_SIZE]{};
  bool _debug;
  uint8_t _sys_id;
  uint8_t _seq_number = 0;
};

/**
 * UART transport module.
 */
class UARTTransporter : public Transporter
{
public:
  UARTTransporter(
    const char * uart_name,
    const uint32_t baudrate,
    const int32_t poll_ms,
    const bool hw_flow_control,
    const bool sw_flow_control,
    const uint8_t sys_id,
    const bool debug);
  virtual ~UARTTransporter();

  int init();
  uint8_t close();

protected:
  ssize_t _read(void * buffer, size_t len);
  ssize_t _write(void * buffer, size_t len);

  bool fds_OK();

  bool baudrate_to_speed(uint32_t bauds, speed_t * speed);

  int _uart_fd;
  int _term_pipe_fds[2] = {-1, -1};
  char _uart_name[64]{};
  uint32_t _baudrate;
  int32_t _poll_ms;
  bool _hw_flow_control{false};
  bool _sw_flow_control{false};
  struct pollfd _poll_fd[2] {};
};

/**
 * UDP transport module.
 */
class UDPTransporter : public Transporter
{
public:
  UDPTransporter(
    const char * udp_ip,
    uint16_t udp_port_recv,
    uint16_t udp_port_send,
    const uint8_t sys_id,
    const bool debug);
  virtual ~UDPTransporter();

  int init();
  uint8_t close();

protected:
  int init_receiver(uint16_t udp_port);
  int init_sender(uint16_t udp_port);

  ssize_t _read(void * buffer, size_t len);
  ssize_t _write(void * buffer, size_t len);

  bool fds_OK();

  int _sender_fd;
  int _receiver_fd;
  char _udp_ip[16]{};
  uint16_t _udp_port_recv;
  uint16_t _udp_port_send;
  struct sockaddr_in _sender_outaddr;
  struct sockaddr_in _receiver_inaddr;
  struct sockaddr_in _receiver_outaddr;
};

} // namespace MicroRTPSAgent

#endif // MICRORTPS_AGENT__TRANSPORT_HPP
