/**
 * Transporters implementation.
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

#include <cstdlib>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>

#include <transport/transport.hpp>

namespace flight_stack
{

/**
 * CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1).
 */
uint16_t const crc16_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**
 * @brief Builds a basic transport module.
 *
 * @param sys_id The system ID of this node.
 * @param debug Whether to print debug messages.
 */
Transporter::Transporter(const uint8_t sys_id, const bool debug)
: _rx_buff_pos(0),
  _debug(debug),
  _sys_id(sys_id)
{}

/**
 * @brief Destroys a basic transport module.
 */
Transporter::~Transporter()
{}

/**
 * @brief Computes the CRC16 of a byte.
 *
 * @param crc The current CRC value.
 * @param data The byte to add to the CRC.
 *
 * @return The new CRC value.
 */
uint16_t Transporter::crc16_byte(uint16_t crc, const uint8_t data)
{
  return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

/**
 * @brief Computes the CRC16 of a buffer.
 *
 * @param buffer The buffer to compute the CRC16 of.
 * @param len The length of the buffer.
 *
 * @return The CRC16 of the buffer.
 */
uint16_t Transporter::crc16(uint8_t const * buffer, size_t len)
{
  uint16_t crc = 0;

  while (len--) {
    crc = crc16_byte(crc, *buffer++);
  }

  return crc;
}

/**
 * @brief Reads data from the transport layer.
 *
 * @param topic_id Pointer to the topic ID to set.
 * @param out_buffer The buffer to write the data to.
 * @param buffer_len The length of the buffer.
 *
 * @return The number of bytes read, or -1 on error.
 */
ssize_t Transporter::read(uint8_t * topic_id, char out_buffer[], size_t buffer_len)
{
  if (nullptr == out_buffer || nullptr == topic_id || !fds_OK()) {
    return -1;
  }

  *topic_id = 255;

  ssize_t len = _read((void *)(_rx_buffer + _rx_buff_pos), sizeof(_rx_buffer) - _rx_buff_pos);

  if (len < 0) {
    int errsv = errno;

    if (errsv && errsv != ETIMEDOUT) {
      if (_debug) {
        RCLCPP_ERROR(
          rclcpp::get_logger("micrortps_transport"),
          "Transporter::read: Read error (%d)",
          errsv);
      }
    }

    return len;
  }

  _rx_buff_pos += len;

  // We read some...
  size_t header_size = sizeof(struct Header);

  // ... but not enough
  if (_rx_buff_pos < header_size) {
    return 0;
  }

  // Check for start
  uint32_t msg_start_pos = 0;
  for (msg_start_pos = 0; msg_start_pos <= _rx_buff_pos - header_size; ++msg_start_pos) {
    if ('>' == _rx_buffer[msg_start_pos] && memcmp(_rx_buffer + msg_start_pos, ">>>", 3) == 0) {
      break;
    }
  }

  // Start not found
  if (msg_start_pos > (_rx_buff_pos - header_size)) {
    if (_debug) {
      RCLCPP_INFO(
        rclcpp::get_logger("micrortps_transport"),
        "Transporter::read: Start not found (↓↓ %" PRIu32 ")",
        msg_start_pos);
    }

    // All we've checked so far is garbage: drop it, but save unchecked bytes
    memmove(_rx_buffer, _rx_buffer + msg_start_pos, _rx_buff_pos - msg_start_pos);
    _rx_buff_pos -= msg_start_pos;
    return -1;
  }

  // [>,>,>,topic_id,sys_id,seq,payload_length_H,payload_length_L,CRCHigh,CRCLow,payloadStart, ... ,payloadEnd]
  struct Header * header = (struct Header *)&_rx_buffer[msg_start_pos];
  uint32_t payload_len = ((uint32_t)header->payload_len_h << 8) | header->payload_len_l;

  // The received message comes from this system: discard it
  // This might happen when:
  //   1. The same UDP port is being used to send a rcv packets or
  //   2. The same topic on the agent is being used for outgoing and incoming data
  if (header->sys_id == _sys_id) {
    // Drop the message and continue with the read buffer
    memmove(_rx_buffer, _rx_buffer + msg_start_pos + 1, _rx_buff_pos - (msg_start_pos + 1));
    _rx_buff_pos -= (msg_start_pos + 1);
    return -1;
  }

  // The message won't fit the buffer
  if (buffer_len < header_size + payload_len) {
    // Drop the message and continue with the read buffer
    memmove(_rx_buffer, _rx_buffer + msg_start_pos + 1, _rx_buff_pos - (msg_start_pos + 1));
    _rx_buff_pos -= (msg_start_pos + 1);
    return -EMSGSIZE;
  }

  // We do not have a complete message yet
  if (msg_start_pos + header_size + payload_len > _rx_buff_pos) {
    // If there's garbage at the beginning, drop it
    if (msg_start_pos > 0) {
      if (_debug) {
        RCLCPP_ERROR(
          rclcpp::get_logger("micrortps_transport"),
          "Transporter::read: Garbage at the beginning of the message (↓ %" PRIu32 ")",
          msg_start_pos);
      }
      memmove(_rx_buffer, _rx_buffer + msg_start_pos, _rx_buff_pos - msg_start_pos);
      _rx_buff_pos -= msg_start_pos;
    }

    return 0;
  }

  uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
  uint16_t calc_crc = crc16((uint8_t *)_rx_buffer + msg_start_pos + header_size, payload_len);

  if (read_crc != calc_crc) {
    if (_debug) {
      RCLCPP_ERROR(
        rclcpp::get_logger("micrortps_transport"),
        "Transporter::read: Bad CRC: %" PRIu16 " != %" PRIu16 "\t\t(↓ %lu)",
        read_crc,
        calc_crc,
        (unsigned long)(header_size + payload_len));
    }

    // Drop garbage up just beyond the start of the message
    memmove(_rx_buffer, _rx_buffer + (msg_start_pos + 1), _rx_buff_pos);

    // If there is a CRC error, the payload len cannot be trusted
    _rx_buff_pos -= (msg_start_pos + 1);

    len = -1;

  } else {
    // Copy message to outbuffer and set other return values
    memmove(out_buffer, _rx_buffer + msg_start_pos + header_size, payload_len);
    *topic_id = header->topic_id;
    len = payload_len + header_size;

    // Discard message from _rx_buffer
    _rx_buff_pos -= msg_start_pos + header_size + payload_len;
    memmove(_rx_buffer, _rx_buffer + msg_start_pos + header_size + payload_len, _rx_buff_pos);
  }

  return len;
}

/**
 * @brief Write data to the transport.
 *
 * @param topic_id Topic ID.
 * @param buffer Buffer to write.
 * @param length Length of the buffer.
 *
 * @return Number of bytes written.
 */
ssize_t Transporter::write(const uint8_t topic_id, char buffer[], size_t length)
{
  if (!fds_OK()) {
    return -1;
  }

  static struct Header header = {{'>', '>', '>'}, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

  // [>,>,>,topic_id,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]
  uint16_t crc = crc16((uint8_t *)&buffer[sizeof(header)], length);

  header.topic_id = topic_id;
  header.sys_id = _sys_id;
  header.seq = _seq_number++;
  header.payload_len_h = (length >> 8) & 0xff;
  header.payload_len_l = length & 0xff;
  header.crc_h = (crc >> 8) & 0xff;
  header.crc_l = crc & 0xff;

  // Headroom for header is created in client
  // Fill in the header in the same payload buffer to call a single node_write
  memcpy(buffer, &header, sizeof(header));
  ssize_t len = _write(buffer, length + sizeof(header));

  if (len != ssize_t(length + sizeof(header))) {
    return len;
  }

  return len + sizeof(header);
}

/**
 * @brief Constructor for the UART Transporter.
 *
 * @param uart_name UART device name.
 * @param baudrate Baudrate.
 * @param poll_ms Polling period in milliseconds.
 * @param hw_flow_control Hardware flow control flag.
 * @param sw_flow_control Software flow control flag.
 * @param sys_id System ID.
 * @param debug Debug flag.
 */
UARTTransporter::UARTTransporter(
  const char * uart_name,
  const uint32_t baudrate,
  const int32_t poll_ms,
  const bool hw_flow_control,
  const bool sw_flow_control,
  const uint8_t sys_id,
  const bool debug)
: Transporter(sys_id, debug),
  _uart_fd(-1),
  _baudrate(baudrate),
  _poll_ms(poll_ms),
  _hw_flow_control(hw_flow_control),
  _sw_flow_control(sw_flow_control)
{
  if (nullptr != uart_name) {
    strcpy(_uart_name, uart_name);
  }
}

/**
 * @brief Destructor for the UART Transporter.
 */
UARTTransporter::~UARTTransporter()
{
  close();
}

/**
 * @brief Initializes the UART Transporter.
 *
 * @return UART file descriptor on success, errno value otherwise.
 */
int UARTTransporter::init()
{
  // Open a serial port
  _uart_fd = open(_uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (_uart_fd < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init: Failed to open device: %s (%d)",
      _uart_name,
      errno);
    return -errno;
  }

  // If using shared UART, no need to set it up
  if (_baudrate == 0) {
    _poll_fd[0].fd = _uart_fd;
    _poll_fd[0].events = POLLIN;
    return _uart_fd;
  }

  // Try to set baud rate
  struct termios uart_config;
  int termios_state;

  // Back up the original uart configuration to restore it after exit
  if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
    int errno_bkp = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init Failed to get configuration of device %s: %d (%d)",
      _uart_name,
      termios_state,
      errno_bkp);
    close();
    return -errno_bkp;
  }

  uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  uart_config.c_lflag &=
    ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

  // Never send SIGTTOU
  uart_config.c_lflag &= ~(TOSTOP);

  // Ignore modem control lines
  uart_config.c_cflag |= CLOCAL;

  // 8 bits
  uart_config.c_cflag |= CS8;

  // Flow control
  if (_hw_flow_control) {
    // HW flow control
    uart_config.c_cflag |= CRTSCTS;
    uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
  } else if (_sw_flow_control) {
    // SW flow control
    uart_config.c_cflag &= ~CRTSCTS;
    uart_config.c_lflag |= (IXON | IXOFF | IXANY);
  } else {
    uart_config.c_cflag &= ~CRTSCTS;
    uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
  }

  // Set baud rate
  speed_t speed;
  if (!baudrate_to_speed(_baudrate, &speed)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init: Failed to set baudrate for device %s: %d",
      _uart_name,
      _baudrate);
    close();
    return -EINVAL;
  }
  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
    int errno_bkp = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init: Failed to set baudrate for device %s: %d (%d)",
      _uart_name,
      termios_state,
      errno_bkp);
    close();
    return -errno_bkp;
  }

  // Apply configuration
  if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
    int errno_bkp = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init: Failed to set configuration of device %s: %d (%d)",
      _uart_name,
      termios_state,
      errno_bkp);
    close();
    return -errno_bkp;
  }

  // For Linux, set high speed polling at the chip level.
  // Since this routine relies on a USB latency change at the chip level,
  // it may fail on certain chip sets if their driver does not support this configuration request.
  {
    struct serial_struct serial_ctl;

    if (ioctl(_uart_fd, TIOCGSERIAL, &serial_ctl) < 0) {
      int errno_bkp = errno;
      RCLCPP_ERROR(
        rclcpp::get_logger("micrortps_transport"),
        "UARTTransporter::init: Failed to read configuration of device %s: (%d)",
        _uart_name,
        errno_bkp);

      if (ioctl(_uart_fd, TCFLSH, TCIOFLUSH) == -1) {
        int errno_bkp = errno;
        RCLCPP_ERROR(
          rclcpp::get_logger("micrortps_transport"),
          "UARTTransporter::init: Failed to flush configuration of device %s: (%d)",
          _uart_name,
          errno_bkp);
        close();
        return -errno_bkp;
      }
    }

    serial_ctl.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(_uart_fd, TIOCSSERIAL, &serial_ctl) < 0) {
      int errno_bkp = errno;
      RCLCPP_ERROR(
        rclcpp::get_logger("micrortps_transport"),
        "UARTTransporter::init: Failed to write serial port latency of device %s: (%d)",
        _uart_name,
        errno_bkp);
      close();
      return -errno_bkp;
    }
  }

  // TODO This seems to cause more problems than it attempts to solve, why is this here?
  /*char aux[64];
  bool flush = false;

  while (0 < ::read(_uart_fd, (void *)&aux, 64)) {
    flush = true;
    usleep(1000);
  }

  if (flush) {
    if (_debug) {
      RCLCPP_INFO(
        rclcpp::get_logger("micrortps_transport"),
        "UARTTransporter::init: Flush");
    }
  } else {
    if (_debug) {
      RCLCPP_INFO(
        rclcpp::get_logger("micrortps_transport"),
        "UARTTransporter::init: No flush");
    }
  }*/

  // Configure poll data for UART fd
  _poll_fd[0].fd = _uart_fd;
  _poll_fd[0].events = POLLIN;

  // Open the termination notice pipe
  if (pipe(_term_pipe_fds)) {
    int errno_bkp = errno;
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::init: Failed to open termination pipe: (%d)",
      errno_bkp);
    close();
    return -errno_bkp;
  }
  _poll_fd[1].fd = _term_pipe_fds[0];
  _poll_fd[1].events = POLLIN;

  return _uart_fd;
}

/**
 * @brief Check if the file descriptors are OK.
 *
 * @return true if the file descriptors are OK.
 */
bool UARTTransporter::fds_OK()
{
  return -1 != _uart_fd;
}

/**
 * @brief Closes the UART.
 *
 * @return 0.
*/
uint8_t UARTTransporter::close()
{
  if (-1 != _uart_fd) {
    char close = CHAR_MAX;
    if (::close(_uart_fd) ||
      (::write(_term_pipe_fds[1], &close, 1) != 1))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("micrortps_transport"),
        "UARTTransporter::close: Failed to close UART");
      return -1;
    }
    _uart_fd = -1;
    RCLCPP_WARN(
      rclcpp::get_logger("micrortps_transport"),
      "UARTTransporter::close: Closed UART");
  }

  return 0;
}

/**
 * @brief Reads from the UART.
 *
 * @param buffer Buffer to read into.
 * @param len Length of the buffer
 *
 * @return Number of bytes read.
 */
ssize_t UARTTransporter::_read(void * buffer, size_t len)
{
  if (nullptr == buffer || !fds_OK()) {
    return -1;
  }

  ssize_t ret = 0;
  int r = poll(_poll_fd, 2, _poll_ms);

  // We are being terminated
  if (_poll_fd[1].revents & POLLIN) {
    return -1;
  }

  // poll failed for some reason
  if (r < 0) {
    return -1;
  }

  // poll got an error
  if (r == 1 &&
    (_poll_fd[0].revents & (POLLERR | POLLHUP | POLLNVAL)))
  {
    return -1;
  }

  // There is new data to read
  if (r == 1 && (_poll_fd[0].revents & POLLIN)) {
    ret = ::read(_uart_fd, buffer, len);
  }

  return ret;
}

/**
 * @brief Writes to the UART.
 *
 * @param buffer Buffer to write from.
 * @param len Length of the buffer.
 *
 * @return Number of bytes written.
 */
ssize_t UARTTransporter::_write(void * buffer, size_t len)
{
  if (nullptr == buffer || !fds_OK()) {
    return -1;
  }

  return ::write(_uart_fd, buffer, len);
}

/**
 * @brief Conversion wrapper for UART speed types.
 *
 * @param bauds Baudrate to convert.
 * @param speed speed_t bitmask to set.
 *
 * @return true if the baudrate was converted.
 */
bool UARTTransporter::baudrate_to_speed(uint32_t bauds, speed_t * speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

  switch (bauds) {
    case 0:      *speed = B0;               break;

    case 50:     *speed = B50;              break;

    case 75:     *speed = B75;              break;

    case 110:    *speed = B110;             break;

    case 134:    *speed = B134;             break;

    case 150:    *speed = B150;             break;

    case 200:    *speed = B200;             break;

    case 300:    *speed = B300;             break;

    case 600:    *speed = B600;             break;

    case 1200:   *speed = B1200;            break;

    case 1800:   *speed = B1800;            break;

    case 2400:   *speed = B2400;            break;

    case 4800:   *speed = B4800;            break;

    case 9600:   *speed = B9600;            break;

    case 19200:  *speed = B19200;           break;

    case 38400:  *speed = B38400;           break;

    case 57600:  *speed = B57600;           break;

    case 115200: *speed = B115200;          break;

    case 230400: *speed = B230400;          break;

    case 460800: *speed = B460800;          break;

    case 500000: *speed = B500000;          break;

    case 921600: *speed = B921600;          break;

    case 1000000: *speed = B1000000;        break;

    case 1500000: *speed = B1500000;        break;

    case 2000000: *speed = B2000000;        break;

#ifdef B3000000
    case 3000000: *speed = B3000000;        break;

#endif
#ifdef B3500000
    case 3500000: *speed = B3500000;        break;

#endif
#ifdef B4000000
    case 4000000: *speed = B4000000;        break;

#endif
    default:
      return false;
  }

  return true;
}

/**
 * @brief Constructor for the UDP transport module.
 *
 * @param udp_ip IP address to use.
 * @param udp_port_recv Port to receive from.
 * @param udp_port_send Port to send to.
 * @param sys_id System ID to use.
 * @param debug Debug mode.
 */
UDPTransporter::UDPTransporter(
  const char * udp_ip,
  uint16_t udp_port_recv,
  uint16_t udp_port_send,
  const uint8_t sys_id,
  const bool debug)
: Transporter(sys_id, debug),
  _sender_fd(-1),
  _receiver_fd(-1),
  _udp_port_recv(udp_port_recv),
  _udp_port_send(udp_port_send)
{
  if (nullptr != udp_ip) {
    strcpy(_udp_ip, udp_ip);
  }
}

/**
 * @brief Destructor for the UDP transport module.
 */
UDPTransporter::~UDPTransporter()
{
  close();
}

/**
 * @brief Initializes the UDP transport module.
 *
 * @return 0 on success, -1 on error.
 */
int UDPTransporter::init()
{
  if (0 > init_receiver(_udp_port_recv) || 0 > init_sender(_udp_port_send)) {
    return -1;
  }

  return 0;
}

/**
 * @brief Checks if the file descriptors are valid.
 *
 * @return true if the file descriptors are valid.
 */
bool UDPTransporter::fds_OK()
{
  return -1 != _sender_fd && -1 != _receiver_fd;
}

/**
 * @brief Initializes the UDP receiver.
 *
 * @param udp_port Port to receive from.
 *
 * @return 0 on success, -1 on error.
 */
int UDPTransporter::init_receiver(uint16_t udp_port)
{
  // Set UDP socket data structure
  memset((char *)&_receiver_inaddr, 0, sizeof(_receiver_inaddr));
  _receiver_inaddr.sin_family = AF_INET;
  _receiver_inaddr.sin_port = htons(udp_port);
  _receiver_inaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  // Create UDP socket
  if ((_receiver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::init_receiver: Socket creation failed");
    return -1;
  }

  // Bind UDP socket
  if (bind(_receiver_fd, (struct sockaddr *)&_receiver_inaddr, sizeof(_receiver_inaddr)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::init_receiver: Bind failed");
    return -1;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("micrortps_transport"),
    "UDPTransporter::init_receiver: Bound to port %d",
    udp_port);

  return 0;
}

/**
 * @brief Initializes the UDP sender.
 *
 * @param udp_port Port to send to.
 *
 * @return 0 on success, -1 on error.
 */
int UDPTransporter::init_sender(uint16_t udp_port)
{

  if ((_sender_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::init_sender: Socket creation failed");
    return -1;
  }

  // Configure sender UDP data structure
  memset((char *)&_sender_outaddr, 0, sizeof(_sender_outaddr));
  _sender_outaddr.sin_family = AF_INET;
  _sender_outaddr.sin_port = htons(udp_port);
  if (inet_aton(_udp_ip, &_sender_outaddr.sin_addr) == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::init_sender: inet_aton() failed");
    return -1;
  }

  return 0;
}

/**
 * @brief Closes the UDP transport module.
 *
 * @return 0.
 */
uint8_t UDPTransporter::close()
{
  if (_sender_fd != -1) {
    shutdown(_sender_fd, SHUT_RDWR);
    ::close(_sender_fd);
    _sender_fd = -1;
    RCLCPP_WARN(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::close: Closed sender socket");
  }

  if (_receiver_fd != -1) {
    shutdown(_receiver_fd, SHUT_RDWR);
    ::close(_receiver_fd);
    _receiver_fd = -1;
    RCLCPP_WARN(
      rclcpp::get_logger("micrortps_transport"),
      "UDPTransporter::close: Closed receiver socket");
  }

  return 0;
}

/**
 * @brief Reads data from the UDP transport module (blocking call).
 *
 * @param buffer Buffer to read into.
 * @param len Length of the buffer.
 *
 * @return Number of bytes read.
 */
ssize_t UDPTransporter::_read(void * buffer, size_t len)
{
  if (nullptr == buffer || !fds_OK()) {
    return -1;
  }

  ssize_t ret = 0;
  static socklen_t addrlen = sizeof(_receiver_outaddr);
  ret = recvfrom(_receiver_fd, buffer, len, 0, (struct sockaddr *)&_receiver_outaddr, &addrlen);
  return ret;
}

/**
 * @brief Writes data to the UDP transport module.
 *
 * @param buffer Buffer to write from.
 * @param len Length of the buffer.
 *
 * @return Number of bytes written.
 */
ssize_t UDPTransporter::_write(void * buffer, size_t len)
{
  if (nullptr == buffer || !fds_OK()) {
    return -1;
  }

  ssize_t ret = 0;
  ret = sendto(
    _sender_fd,
    buffer,
    len,
    0,
    (struct sockaddr *)&_sender_outaddr,
    sizeof(_sender_outaddr));
  return ret;
}

} // namespace flight_stack
