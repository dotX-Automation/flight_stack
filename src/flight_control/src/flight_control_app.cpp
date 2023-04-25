/**
 * Flight Control standalone application.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 24, 2022
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define MODULE_NAME "flight_control_app"
#define UNUSED(arg) (void)(arg)

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <iostream>

#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <ros2_signal_handler/signal_handler.hpp>

#include <flight_control/flight_control.hpp>

using namespace FlightControl;
using namespace ROS2SignalHandler;

/**
 * @brief Signal handler routine, terminates this module.
 *
 * @param sig Signal.
 * @param logger_name Name of the logger to use.
 * @param executor Executor to cancel.
 */
void sig_handler(
  int sig,
  std::string & logger_name,
  rclcpp::Executor::SharedPtr executor)
{
  UNUSED(sig);
  RCLCPP_DEBUG(
    rclcpp::get_logger(logger_name),
    "Terminating %s",
    MODULE_NAME);
  executor->cancel();
}

int main(int argc, char ** argv)
{
  // Disable I/O buffering
  if (setvbuf(stdout, NULL, _IONBF, 0)) {
    RCLCPP_FATAL(
      rclcpp::get_logger(MODULE_NAME),
      "Failed to set I/O buffering");
    exit(EXIT_FAILURE);
  }

  // Create and initialize ROS 2 context
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options = rclcpp::InitOptions();
  init_options.shutdown_on_sigint = true;
  context->init(argc, argv, init_options);

  // Initialize ROS 2 node
  rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
  node_opts.context(context);
  auto flight_control_node = std::make_shared<FlightControlNode>(node_opts);

  // Create and configure executor
  rclcpp::ExecutorOptions executor_opts = rclcpp::ExecutorOptions();
  executor_opts.context = context;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(executor_opts);
  executor->add_node(flight_control_node);

  // Initialize and configure signal handler
  SignalHandler & signal_handler = SignalHandler::get_global_signal_handler();
  signal_handler.init(
    context,
    MODULE_NAME "_signal_handler",
    std::bind(
      sig_handler,
      std::placeholders::_1,
      std::placeholders::_2,
      executor));
  signal_handler.install(SIGINT);
  signal_handler.install(SIGTERM);
  signal_handler.ignore(SIGHUP);

  RCLCPP_WARN(
    rclcpp::get_logger(MODULE_NAME),
    "(%d) " MODULE_NAME " online",
    getpid());

  // Spin the executor
  executor->spin();

  // Destroy ROS 2 node
  flight_control_node.reset();

  // Finalize signal handler
  signal_handler.fini();

  exit(EXIT_SUCCESS);
}
