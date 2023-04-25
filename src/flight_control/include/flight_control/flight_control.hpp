/**
 * Flight Control node definition.
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

#ifndef STANIS_FLIGHT_CONTROL_HPP
#define STANIS_FLIGHT_CONTROL_HPP

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
#include <thread>
#include <vector>

#include <pthread.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/qos_profiles.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/log_message.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/px4_timestamp.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position_stamped.hpp>
#include <stanis_interfaces/msg/command_result.hpp>
#include <stanis_interfaces/msg/pose.hpp>
#include <stanis_interfaces/msg/position_setpoint.hpp>
#include <stanis_interfaces/msg/velocity_setpoint.hpp>
#include <std_msgs/msg/header.hpp>

#include <stanis_interfaces/srv/reset.hpp>
#include <stanis_interfaces/srv/setpoints_switch.hpp>

#include <stanis_interfaces/action/arm.hpp>
#include <stanis_interfaces/action/disarm.hpp>
#include <stanis_interfaces/action/landing.hpp>
#include <stanis_interfaces/action/reach.hpp>
#include <stanis_interfaces/action/takeoff.hpp>
#include <stanis_interfaces/action/turn.hpp>

#include <stanis_qos/flight_control_qos.hpp>

using namespace px4_msgs::msg;
using namespace rcl_interfaces::msg;
using namespace stanis_interfaces::action;
using namespace stanis_interfaces::msg;
using namespace stanis_interfaces::srv;

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

namespace FlightControl
{

/**
 * Drone pose.
 */
struct DronePose
{
  Eigen::Vector3f position = {0.0f, 0.0f, 0.0f}; // m
  Eigen::Vector3f velocity = {0.0f, 0.0f, 0.0f}; // m/s
  Eigen::Quaternionf attitude = Eigen::Quaternionf::Identity();
  Eigen::EulerAnglesXYZf rpy = {0.0f, 0.0f, 0.0f}; // rad, [-PI +PI]

  DronePose() {}

  DronePose(float pose_x, float pose_y, float pose_z, float pose_yaw)
  : position(Eigen::Vector3f(pose_x, pose_y, pose_z)),
    attitude(
      Eigen::Quaternionf(
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(pose_yaw, Eigen::Vector3f::UnitZ()))),
    rpy(Eigen::Vector3f(0.0f, 0.0f, pose_yaw))
  {}

  DronePose(const geometry_msgs::msg::Point32 & coordinates, float heading)
  : position(Eigen::Vector3f(coordinates.x, coordinates.y, coordinates.z)),
    attitude(
      Eigen::Quaternionf(
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitZ()))),
    rpy(Eigen::Vector3f(0.0f, 0.0f, heading))
  {}

  DronePose(
    Eigen::Vector3f & pos,
    Eigen::Vector3f & vel,
    Eigen::Quaternionf & q,
    Eigen::EulerAnglesXYZf & rpy_angles)
  : position(pos),
    velocity(vel),
    attitude(q),
    rpy(rpy_angles)
  {}
};

/**
 * FMU OFFBOARD control modes.
 */
enum ControlModes : int
{
  POSITION,
  VELOCITY
};

/**
 * FMU setpoint.
 */
struct Setpoint
{
  ControlModes control_mode = ControlModes::POSITION;
  float x = 0.0f; // m
  float y = 0.0f; // m
  float z = 0.0f; // m
  float yaw = 0.0f; // rad
  float vx = 0.0f; // m/s
  float vy = 0.0f; // m/s
  float vz = 0.0f; // m/s
  float vyaw = 0.0f; // rad/s

  Setpoint() {}

  Setpoint(
    ControlModes mode,
    float setp_x, float setp_y, float setp_z,
    float setp_yaw,
    float setp_vx, float setp_vy, float setp_vz, float setp_vyaw)
  {
    control_mode = mode;
    x = setp_x;
    y = setp_y;
    z = setp_z;
    yaw = setp_yaw;
    vx = setp_vx;
    vy = setp_vy;
    vz = setp_vz;
    vyaw = setp_vyaw;
  }

  Setpoint(float setp_x, float setp_y, float setp_z, float setp_yaw)
  : control_mode(ControlModes::POSITION),
    x(setp_x),
    y(setp_y),
    z(setp_z),
    yaw(setp_yaw)
  {}
};

/**
 * Low-level flight operations module, interfaced with FMU.
 */
class FlightControlNode : public rclcpp::Node
{
public:
  FlightControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~FlightControlNode();

private:
  /* Node initialization routines */
  void init_atomics();
  void init_cgroups();
  void init_sync_primitives();
  void init_parameters();
  void init_subscriptions();
  void init_publishers();
  void init_msg_filters();
  void init_services();
  void init_actions();

  /* Timers callback groups */
  rclcpp::CallbackGroup::SharedPtr setpoints_timer_cgroup_;

  /* Timers */
  rclcpp::TimerBase::SharedPtr setpoints_timer_;

  /* Timer callback */
  void setpoints_timer_callback();

  /* Topic subscriptions callback groups */
  rclcpp::CallbackGroup::SharedPtr battery_status_cgroup_;
  rclcpp::CallbackGroup::SharedPtr log_message_cgroup_;
  rclcpp::CallbackGroup::SharedPtr position_setpoint_cgroup_;
  rclcpp::CallbackGroup::SharedPtr px4_timestamp_cgroup_;
  rclcpp::CallbackGroup::SharedPtr takeoff_status_cgroup_;
  rclcpp::CallbackGroup::SharedPtr vehicle_command_ack_cgroup_;
  rclcpp::CallbackGroup::SharedPtr velocity_setpoint_cgroup_;

  /* Topic subscriptions */
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_sub_;
  rclcpp::Subscription<LogMessage>::SharedPtr log_message_sub_;
  rclcpp::Subscription<PositionSetpoint>::SharedPtr position_setpoint_sub_;
  rclcpp::Subscription<PX4Timestamp>::SharedPtr px4_timestamp_sub_;
  rclcpp::Subscription<TakeoffStatus>::SharedPtr takeoff_status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::Subscription<VelocitySetpoint>::SharedPtr velocity_setpoint_sub_;

  /* Topic subscriptions callbacks */
  void battery_status_callback(const BatteryStatus::SharedPtr msg);
  void log_message_callback(const LogMessage::SharedPtr msg);
  void position_setpoint_callback(const PositionSetpoint::SharedPtr msg);
  void px4_timestamp_callback(const PX4Timestamp::SharedPtr msg);
  void vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg);
  void velocity_setpoint_callback(const VelocitySetpoint::SharedPtr msg);
  void takeoff_status_callback(const TakeoffStatus::SharedPtr msg);

  /* Pose message filter subscribers */
  std::shared_ptr<message_filters::Subscriber<VehicleLocalPositionStamped>> local_pos_sub_;
  std::shared_ptr<message_filters::Subscriber<VehicleAttitudeStamped>> attitude_sub_;

  /* Pose message filter synchronizer */
  std::shared_ptr<message_filters::Synchronizer<pose_sync_policy>> pose_synchronizer_;

  /* Pose message filter callback */
  void pose_callback(
    const VehicleLocalPositionStamped::SharedPtr & local_position_msg,
    const VehicleAttitudeStamped::SharedPtr & attitude_msg);

  /* Topic publishers */
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_pose_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

  /* Services callback groups */
  rclcpp::CallbackGroup::SharedPtr reset_cgroup_;
  rclcpp::CallbackGroup::SharedPtr setpoints_switch_cgroup_;

  /* Service servers */
  rclcpp::Service<Reset>::SharedPtr reset_server_;
  rclcpp::Service<SetpointsSwitch>::SharedPtr setpoints_switch_server_;

  /* Services callbacks */
  void reset_callback(
    Reset::Request::SharedPtr req,
    Reset::Response::SharedPtr resp);
  void setpoints_switch_callback(
    SetpointsSwitch::Request::SharedPtr req,
    SetpointsSwitch::Response::SharedPtr resp);

  /* Actions callback group */
  rclcpp::CallbackGroup::SharedPtr actions_cgroup_;

  /* Action servers options */
  rcl_action_server_options_t arm_server_options_{};
  rcl_action_server_options_t disarm_server_options_{};
  rcl_action_server_options_t landing_server_options_{};
  rcl_action_server_options_t reach_server_options_{};
  rcl_action_server_options_t takeoff_server_options_{};
  rcl_action_server_options_t turn_server_options_{};

  /* Action servers */
  rclcpp_action::Server<Arm>::SharedPtr arm_server_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;
  rclcpp_action::Server<Landing>::SharedPtr landing_server_;
  rclcpp_action::Server<Reach>::SharedPtr reach_server_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<Turn>::SharedPtr turn_server_;

  /* Actions goal request handlers */
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

  /* Actions cancellation request handlers */
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

  /* Actions goal acceptance handlers */
  void handle_arm_accepted(const ArmGoalHandleSharedPtr goal_handle);
  void handle_disarm_accepted(const DisarmGoalHandleSharedPtr goal_handle);
  void handle_landing_accepted(const LandingGoalHandleSharedPtr goal_handle);
  void handle_reach_accepted(const ReachGoalHandleSharedPtr goal_handle);
  void handle_takeoff_accepted(const TakeoffGoalHandleSharedPtr goal_handle);
  void handle_turn_accepted(const TurnGoalHandleSharedPtr goal_handle);

  /* Flight routines */
  void arm(const ArmGoalHandleSharedPtr goal_handle);
  void disarm(const DisarmGoalHandleSharedPtr goal_handle);
  void landing(const LandingGoalHandleSharedPtr goal_handle);
  void reach(const ReachGoalHandleSharedPtr goal_handle);
  void takeoff(const TakeoffGoalHandleSharedPtr goal_handle);
  void turn(const TurnGoalHandleSharedPtr goal_handle);

  /* Synchronization primitives for internal update operations */
  pthread_spinlock_t state_lock_;
  pthread_spinlock_t setpoint_lock_;
  pthread_spinlock_t operation_lock_;
  std::mutex fmu_cmd_ack_lock_;
  std::mutex takeoff_status_lock_;
  std::condition_variable fmu_cmd_ack_cv_;
  std::condition_variable takeoff_status_cv_;
  std::atomic<bool> fmu_cmd_ack_received_;
  std::atomic<bool> takeoff_status_received_;

  /* Internal state variables */
  std::atomic<bool> armed_;
  std::atomic<bool> airborne_;
  DronePose drone_pose_{};
  std::atomic<bool> fmu_cmd_success_;
  uint8_t last_takeoff_status_ = TakeoffStatus::TAKEOFF_STATE_UNINITIALIZED;
  bool low_battery_ = false;
  rclcpp::Time low_battery_timer_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  rclcpp::Time last_pose_timestamp_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  /* Setpoint for FMU */
  Setpoint fmu_setpoint_{};

  /* Valid FMU timestamp */
  std::atomic<uint64_t> fmu_timestamp_;

  /* Node parameters */
  int64_t fmu_command_attempts_ = 0; // ms
  int64_t fmu_command_timeout_ = 0; // ms
  int64_t landing_timeout_ = 0; // ms
  double low_battery_voltage_ = 0.0; // V
  bool monitor_battery_ = false;
  double roll_pitch_stabilization_confidence_ = 0.0; // rad
  int64_t setpoints_period_ = 0; // ms
  double takeoff_position_confidence_ = 0.0; // m
  int64_t takeoff_timeout_ = 0; // ms
  int64_t travel_sleep_time_ = 0; // ms
  int64_t turn_sleep_time_ = 0; // ms
  double turn_step_ = 0.0; // rad
  double v_horz_stabilization_max_ = 0.0; // m/s
  double v_vert_stabilization_max_ = 0.0; // m/s
  double yaw_stabilization_confidence_ = 0.0; // rad

  /* Internal steady clock */
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  /* Node parameters descriptors */
  ParameterDescriptor fmu_command_attempts_descriptor_;
  ParameterDescriptor fmu_command_timeout_descriptor_;
  ParameterDescriptor landing_timeout_descriptor_;
  ParameterDescriptor low_battery_voltage_descriptor_;
  ParameterDescriptor monitor_battery_descriptor_;
  ParameterDescriptor roll_pitch_stabilization_confidence_descriptor_;
  ParameterDescriptor setpoints_period_descriptor_;
  ParameterDescriptor takeoff_position_confidence_descriptor_;
  ParameterDescriptor takeoff_timeout_descriptor_;
  ParameterDescriptor travel_sleep_time_descriptor_;
  ParameterDescriptor turn_sleep_time_descriptor_;
  ParameterDescriptor turn_step_descriptor_;
  ParameterDescriptor v_horz_stabilization_max_descriptor_;
  ParameterDescriptor v_vert_stabilization_max_descriptor_;
  ParameterDescriptor yaw_confidence_descriptor_;

  /* Parameters callback */
  OnSetParametersCallbackHandle::SharedPtr on_set_params_chandle_;
  SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /* Utility routines */
  void create_spinlock(pthread_spinlock_t * lock);
  void declare_bool_parameter(
    std::string && name,
    bool default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_double_parameter(
    std::string && name,
    double default_val, double from, double to, double step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_int_parameter(
    std::string && name,
    int64_t default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void activate_setpoints_timer();
  void deactivate_setpoints_timer();
  bool change_setpoint(const Setpoint & new_setpoint);
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
  void stop_drone(const DronePose & pose);
  bool arm_drone(CommandResult & result);
  bool is_stabilized(const DronePose & pose);
  bool is_on_target(
    const Eigen::Vector3f & current,
    const Eigen::Vector3f & target,
    float confidence_radius);
  bool is_oriented(float yaw, float tgt_yaw);
  float get_distance(
    const Eigen::Vector3f & pos1,
    const Eigen::Vector3f & pos2);
  rclcpp_action::ResultCode do_turn(
    const TurnGoalHandleSharedPtr goal_handle,
    Setpoint & turn_setpoint,
    float start_yaw,
    float target_yaw);
};

} // namespace FlightControl

#endif // STANIS_FLIGHT_CONTROL_HPP
