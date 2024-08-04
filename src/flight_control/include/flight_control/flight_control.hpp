/**
 * Flight Control node definition.
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

#ifndef FLIGHT_STACK__FLIGHT_CONTROL_HPP
#define FLIGHT_STACK__FLIGHT_CONTROL_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pose_kit/pose.hpp>
#include <pose_kit/dynamic_pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dua_interfaces/msg/command_result_stamped.hpp>
#include <dua_interfaces/msg/rates_setpoint.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/log_message.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position_stamped.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <std_msgs/msg/header.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <dua_interfaces/action/arm.hpp>
#include <dua_interfaces/action/disarm.hpp>
#include <dua_interfaces/action/landing.hpp>
#include <dua_interfaces/action/reach.hpp>
#include <dua_interfaces/action/takeoff.hpp>
#include <dua_interfaces/action/turn.hpp>

#include "flight_control_types.hpp"

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace px4_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;

using namespace std_srvs::srv;

using namespace dua_interfaces::action;

using ArmGoalHandle = rclcpp_action::ServerGoalHandle<Arm>;
using ArmGoalSharedPtr = std::shared_ptr<const Arm::Goal>;
using ArmGoalHandleSharedPtr = std::shared_ptr<ArmGoalHandle>;

using DisarmGoalHandle = rclcpp_action::ServerGoalHandle<Disarm>;
using DisarmGoalSharedPtr = std::shared_ptr<const Disarm::Goal>;
using DisarmGoalHandleSharedPtr = std::shared_ptr<DisarmGoalHandle>;

using LandingGoalHandle = rclcpp_action::ServerGoalHandle<Landing>;
using LandingGoalSharedPtr = std::shared_ptr<const Landing::Goal>;
using LandingGoalHandleSharedPtr = std::shared_ptr<LandingGoalHandle>;

using ReachGoalHandle = rclcpp_action::ServerGoalHandle<Reach>;
using ReachGoalSharedPtr = std::shared_ptr<const Reach::Goal>;
using ReachGoalHandleSharedPtr = std::shared_ptr<ReachGoalHandle>;

using TakeoffGoalHandle = rclcpp_action::ServerGoalHandle<Takeoff>;
using TakeoffGoalSharedPtr = std::shared_ptr<const Takeoff::Goal>;
using TakeoffGoalHandleSharedPtr = std::shared_ptr<TakeoffGoalHandle>;

using TurnGoalHandle = rclcpp_action::ServerGoalHandle<Turn>;
using TurnGoalSharedPtr = std::shared_ptr<const Turn::Goal>;
using TurnGoalHandleSharedPtr = std::shared_ptr<TurnGoalHandle>;

typedef message_filters::sync_policies::ApproximateTime<px4_msgs::msg::VehicleLocalPositionStamped,
    px4_msgs::msg::VehicleAttitudeStamped> pose_sync_policy;

namespace flight_stack
{

/**
 * Low-level flight operations module, abstracts a PX4-based FMU.
 */
class FlightControlNode : public dua_node::NodeBase
{
public:
  FlightControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~FlightControlNode();

private:
  /* Node initialization routines. */
  void init_atomics();
  void init_cgroups();
  void init_parameters();
  void init_subscriptions();
  void init_tf2();
  void init_publishers();
  void init_msg_filters();
  void init_services();
  void init_actions();

  /* Timers callback groups. */
  rclcpp::CallbackGroup::SharedPtr setpoints_timer_cgroup_;

  /* Timers. */
  rclcpp::TimerBase::SharedPtr setpoints_timer_;

  /* Timer callbacks. */
  void setpoints_timer_callback();

  /* TF2 data. */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Topic subscriptions callback groups. */
  rclcpp::CallbackGroup::SharedPtr battery_state_cgroup_;
  rclcpp::CallbackGroup::SharedPtr log_message_cgroup_;
  rclcpp::CallbackGroup::SharedPtr odometry_cgroup_;
  rclcpp::CallbackGroup::SharedPtr position_setpoint_cgroup_;
  rclcpp::CallbackGroup::SharedPtr setpoint_stream_cgroup_;
  rclcpp::CallbackGroup::SharedPtr takeoff_status_cgroup_;
  rclcpp::CallbackGroup::SharedPtr vehicle_command_ack_cgroup_;
  rclcpp::CallbackGroup::SharedPtr velocity_setpoint_cgroup_;

  /* Topic subscriptions. */
  rclcpp::Subscription<BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<LogMessage>::SharedPtr log_message_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr position_setpoint_sub_;
  rclcpp::Subscription<RatesSetpoint>::SharedPtr rates_stream_sub_;
  rclcpp::Subscription<TakeoffStatus>::SharedPtr takeoff_status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::Subscription<Twist>::SharedPtr velocity_setpoint_sub_;

  /* Topic subscriptions callbacks. */
  void battery_state_callback(const BatteryState::SharedPtr msg);
  void log_message_callback(const LogMessage::SharedPtr msg);
  void odometry_callback(const Odometry::SharedPtr msg);
  void position_setpoint_callback(const PoseStamped::SharedPtr msg);
  void rates_stream_callback(const RatesSetpoint::SharedPtr msg);
  void vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg);
  void velocity_setpoint_callback(const Twist::SharedPtr msg);
  void takeoff_status_callback(const TakeoffStatus::SharedPtr msg);

  /* Pose message filter subscribers. */
  std::shared_ptr<message_filters::Subscriber<VehicleLocalPositionStamped>> local_pos_sub_;
  std::shared_ptr<message_filters::Subscriber<VehicleAttitudeStamped>> attitude_sub_;

  /* Pose message filter synchronizer. */
  std::shared_ptr<message_filters::Synchronizer<pose_sync_policy>> pose_synchronizer_;

  /* Pose message filter callback. */
  void pose_callback(
    const VehicleLocalPositionStamped::SharedPtr & local_position_msg,
    const VehicleAttitudeStamped::SharedPtr & attitude_msg);

  /* Topic publishers. */
  rclcpp::Publisher<Odometry>::SharedPtr ekf2_odometry_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr ekf2_pose_pub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<VehicleVisualOdometry>::SharedPtr visual_odometry_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_pub_;

  /* Services callback groups. */
  rclcpp::CallbackGroup::SharedPtr reboot_cgroup_;
  rclcpp::CallbackGroup::SharedPtr reset_cgroup_;
  rclcpp::CallbackGroup::SharedPtr setpoints_switch_cgroup_;

  /* Service servers. */
  rclcpp::Service<Trigger>::SharedPtr reboot_server_;
  rclcpp::Service<Trigger>::SharedPtr reset_server_;
  rclcpp::Service<SetBool>::SharedPtr setpoints_switch_server_;

  /* Services callbacks */
  void reboot_callback(
    Trigger::Request::SharedPtr req,
    Trigger::Response::SharedPtr resp);
  void reset_callback(
    Trigger::Request::SharedPtr req,
    Trigger::Response::SharedPtr resp);
  void setpoints_switch_callback(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr resp);

  /* Actions callback group. */
  rclcpp::CallbackGroup::SharedPtr actions_cgroup_;

  /* Action servers. */
  rclcpp_action::Server<Arm>::SharedPtr arm_server_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;
  rclcpp_action::Server<Landing>::SharedPtr landing_server_;
  rclcpp_action::Server<Reach>::SharedPtr reach_server_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<Turn>::SharedPtr turn_server_;

  /* Actions goal request handlers. */
  rclcpp_action::GoalResponse handle_arm_goal(
    const rclcpp_action::GoalUUID & uuid,
    ArmGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_disarm_goal(
    const rclcpp_action::GoalUUID & uuid,
    DisarmGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_landing_goal(
    const rclcpp_action::GoalUUID & uuid,
    LandingGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_reach_goal(
    const rclcpp_action::GoalUUID & uuid,
    ReachGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_takeoff_goal(
    const rclcpp_action::GoalUUID & uuid,
    TakeoffGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_turn_goal(
    const rclcpp_action::GoalUUID & uuid,
    TurnGoalSharedPtr goal);

  /* Actions cancellation request handlers. */
  rclcpp_action::CancelResponse handle_arm_cancel(
    const ArmGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_disarm_cancel(
    const DisarmGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_landing_cancel(
    const LandingGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_reach_cancel(
    const ReachGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_takeoff_cancel(
    const TakeoffGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_turn_cancel(
    const TurnGoalHandleSharedPtr goal_handle);

  /* Actions goal acceptance handlers. */
  void handle_arm_accepted(const ArmGoalHandleSharedPtr goal_handle);
  void handle_disarm_accepted(const DisarmGoalHandleSharedPtr goal_handle);
  void handle_landing_accepted(const LandingGoalHandleSharedPtr goal_handle);
  void handle_reach_accepted(const ReachGoalHandleSharedPtr goal_handle);
  void handle_takeoff_accepted(const TakeoffGoalHandleSharedPtr goal_handle);
  void handle_turn_accepted(const TurnGoalHandleSharedPtr goal_handle);

  /* Flight routines. */
  void arm(const ArmGoalHandleSharedPtr goal_handle);
  void disarm(const DisarmGoalHandleSharedPtr goal_handle);
  void landing(const LandingGoalHandleSharedPtr goal_handle);
  void reach(const ReachGoalHandleSharedPtr goal_handle);
  void takeoff(const TakeoffGoalHandleSharedPtr goal_handle);
  void turn(const TurnGoalHandleSharedPtr goal_handle);

  /* Synchronization primitives for internal update operations. */
  std::mutex fmu_cmd_ack_lock_;
  std::mutex operation_lock_;
  std::mutex setpoint_lock_;
  std::mutex state_lock_;
  std::mutex stream_reset_lock_;
  std::mutex takeoff_status_lock_;
  std::condition_variable fmu_cmd_ack_cv_;
  std::condition_variable takeoff_status_cv_;
  std::atomic<bool> fmu_cmd_ack_received_;
  std::atomic<bool> takeoff_status_received_;

  /* Internal state variables. */
  std::atomic<bool> armed_;
  std::atomic<bool> airborne_;
  std::atomic<bool> de_ascending_;
  pose_kit::DynamicPose drone_pose_{};
  pose_kit::DynamicPose drone_pose_local_{};
  std::atomic<bool> fmu_cmd_success_;
  std::atomic<uint64_t> last_stream_ts_;
  uint8_t last_takeoff_status_ = TakeoffStatus::TAKEOFF_STATE_UNINITIALIZED;
  bool low_battery_ = false;
  rclcpp::Time low_battery_timer_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  rclcpp::Time last_pose_timestamp_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  /* Setpoint for FMU. */
  Setpoint fmu_setpoint_{};

  /* Node parameters. */
  std::string body_frame_ = "";
  std::vector<bool> data_to_px4_ = {true, true, true, true, true, true};
  int64_t fmu_command_attempts_ = 0; // ms
  int64_t fmu_command_timeout_ = 0; // ms
  std::string frame_prefix_ = "";
  std::string global_frame_ = "";
  double landing_step_ = 0.0; // m
  int64_t landing_timeout_ = 0; // ms
  std::string local_frame_ = "";
  double low_battery_voltage_ = 0.0; // V
  bool monitor_battery_ = false;
  bool publish_tf_ = false;
  double roll_pitch_stabilization_confidence_ = 0.0; // rad
  int64_t setpoints_period_ = 0; // ms
  int64_t setpoints_stream_timeout_ = 0; // us
  double takeoff_position_confidence_ = 0.0; // m
  int64_t takeoff_timeout_ = 0; // ms
  double tf2_timeout_ = 0.0; // s
  int64_t travel_sleep_time_ = 0; // ms
  int64_t turn_sleep_time_ = 0; // ms
  double turn_step_ = 0.0; // rad
  bool update_setpoint_ = false;
  double v_horz_stabilization_max_ = 0.0; // m/s
  double v_vert_stabilization_max_ = 0.0; // m/s
  double yaw_stabilization_confidence_ = 0.0; // rad

  /* Node parameters validators. */
  bool validate_data_to_px4(const rclcpp::Parameter & p);
  bool validate_fmu_command_attempts(const rclcpp::Parameter & p);
  bool validate_fmu_command_timeout(const rclcpp::Parameter & p);
  bool validate_landing_step(const rclcpp::Parameter & p);
  bool validate_landing_timeout(const rclcpp::Parameter & p);
  bool validate_roll_pitch_stabilization_confidence(const rclcpp::Parameter & p);
  bool validate_setpoints_period(const rclcpp::Parameter & p);
  bool validate_takeoff_position_confidence(const rclcpp::Parameter & p);
  bool validate_takeoff_timeout(const rclcpp::Parameter & p);
  bool validate_travel_sleep_time(const rclcpp::Parameter & p);
  bool validate_turn_sleep_time(const rclcpp::Parameter & p);
  bool validate_turn_step(const rclcpp::Parameter & p);
  bool validate_v_horz_stabilization_max(const rclcpp::Parameter & p);
  bool validate_v_vert_stabilization_max(const rclcpp::Parameter & p);
  bool validate_yaw_stabilization_confidence(const rclcpp::Parameter & p);

  /* Internal steady clock. */
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  /* Utility routines. */
  inline uint64_t get_time_us() const
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  inline bool check_frame_id_global(const std::string & frame_id) const
  {
    return frame_id == global_frame_;
  }
  inline bool check_frame_id_local(const std::string & frame_id) const
  {
    return frame_id == local_frame_;
  }
  inline bool check_frame_id_body(const std::string & frame_id) const
  {
    return frame_id == body_frame_;
  }
  void activate_setpoints_timer();
  void deactivate_setpoints_timer();
  bool change_setpoint(const Setpoint & new_setpoint, bool to_update = false);
  Setpoint setpoint_global_to_local(const Setpoint & global_setpoint);
  bool send_fmu_command(
    uint16_t cmd,
    float p1 = NAN,
    float p2 = NAN,
    float p3 = NAN,
    float p4 = NAN,
    float p5 = NAN,
    float p6 = NAN,
    float p7 = NAN);
  void notify_takeoff_status();
  void stop_drone();
  bool arm_drone(CommandResultStamped & result);
  bool is_stabilized(const pose_kit::DynamicPose & pose);
  bool is_on_target(
    const Eigen::Vector3d & current,
    const Eigen::Vector3d & target,
    double confidence_radius);
  bool is_oriented(double yaw, double tgt_yaw);
  double get_distance(
    const Eigen::Vector3d & pos1,
    const Eigen::Vector3d & pos2);
  rclcpp_action::ResultCode do_turn(
    const TurnGoalHandleSharedPtr goal_handle,
    Setpoint & turn_setpoint,
    double start_yaw,
    double target_yaw);
};

} // namespace flight_stack

#endif // FLIGHT_STACK__FLIGHT_CONTROL_HPP
