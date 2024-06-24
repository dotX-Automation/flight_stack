/**
 * Flight Control node initialization routines.
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

#include <flight_control/flight_control.hpp>

namespace flight_control
{

/**
 * @brief Flight Control node constructor.
 *
 * @param node_opts Options for the base node.
 */
FlightControlNode::FlightControlNode(const rclcpp::NodeOptions & node_options)
: NodeBase("flight_control", node_options, true)
{
  init_atomics();
  init_cgroups();
  init_parameters();
  init_subscriptions();
  init_tf2();
  init_publishers();
  init_msg_filters();
  init_services();
  init_actions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Flight Control node destructor.
 */
FlightControlNode::~FlightControlNode()
{}

/**
 * @brief Routine to initialize atomic members.
 */
void FlightControlNode::init_atomics()
{
  fmu_cmd_ack_received_.store(true, std::memory_order_release);
  fmu_cmd_success_.store(false, std::memory_order_release);
  takeoff_status_received_.store(true, std::memory_order_release);
  armed_.store(false, std::memory_order_release);
  airborne_.store(false, std::memory_order_release);
  de_ascending_.store(false, std::memory_order_release);
  last_stream_ts_.store(0ULL, std::memory_order_release);
}

/**
 * @brief Routine to initialize callback groups.
 */
void FlightControlNode::init_cgroups()
{
  // Timers
  setpoints_timer_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Topic subscriptions
  battery_state_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  log_message_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  position_setpoint_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  velocity_setpoint_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  takeoff_status_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  vehicle_command_ack_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  setpoint_stream_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Services
  reboot_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  reset_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  setpoints_switch_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Actions
  actions_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void FlightControlNode::init_subscriptions()
{
  // battery_state
  if (this->get_parameter("monitor_battery").as_bool()) {
    auto battery_state_opts = rclcpp::SubscriptionOptions();
    battery_state_opts.callback_group = battery_state_cgroup_;
    battery_state_sub_ = this->create_subscription<BatteryState>(
      agent_node_name_ + "/fmu/battery_state/out",
      DUAQoS::get_datum_qos(),
      std::bind(
        &FlightControlNode::battery_state_callback,
        this,
        std::placeholders::_1),
      battery_state_opts);
  }

  // log_message
  auto log_message_opts = rclcpp::SubscriptionOptions();
  log_message_opts.callback_group = log_message_cgroup_;
  log_message_sub_ = this->create_subscription<LogMessage>(
    agent_node_name_ + "/fmu/log_message/out",
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::log_message_callback,
      this,
      std::placeholders::_1),
    log_message_opts);

  // odometry
  auto odometry_opts = rclcpp::SubscriptionOptions();
  odometry_opts.callback_group = odometry_cgroup_;
  odometry_sub_ = this->create_subscription<Odometry>(
    this->get_parameter("odometry_topic_name").as_string(),
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::odometry_callback,
      this,
      std::placeholders::_1),
    odometry_opts);

  // position_setpoint
  auto position_setpoint_opts = rclcpp::SubscriptionOptions();
  position_setpoint_opts.callback_group = position_setpoint_cgroup_;
  position_setpoint_sub_ = this->create_subscription<PositionSetpoint>(
    "~/position_setpoint",
    DUAQoS::get_command_qos(),
    std::bind(
      &FlightControlNode::position_setpoint_callback,
      this,
      std::placeholders::_1),
    position_setpoint_opts);

  // rates
  auto rates_opts = rclcpp::SubscriptionOptions();
  rates_opts.callback_group = setpoint_stream_cgroup_;
  rates_stream_sub_ = this->create_subscription<RatesSetpoint>(
    "~/rates",
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::rates_stream_callback,
      this,
      std::placeholders::_1),
    rates_opts);

  // velocity_setpoint
  auto velocity_setpoint_opts = rclcpp::SubscriptionOptions();
  velocity_setpoint_opts.callback_group = velocity_setpoint_cgroup_;
  velocity_setpoint_sub_ = this->create_subscription<VelocitySetpoint>(
    "~/velocity_setpoint",
    DUAQoS::get_command_qos(),
    std::bind(
      &FlightControlNode::velocity_setpoint_callback,
      this,
      std::placeholders::_1),
    velocity_setpoint_opts);

  // velocity_stream
  auto velocity_stream_opts = rclcpp::SubscriptionOptions();
  velocity_stream_opts.callback_group = setpoint_stream_cgroup_;
  velocity_stream_sub_ = this->create_subscription<VelocitySetpoint>(
    "~/velocity_stream",
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::velocity_stream_callback,
      this,
      std::placeholders::_1),
    velocity_stream_opts);

  // takeoff_status
  auto takeoff_status_opts = rclcpp::SubscriptionOptions();
  takeoff_status_opts.callback_group = takeoff_status_cgroup_;
  takeoff_status_sub_ = this->create_subscription<TakeoffStatus>(
    agent_node_name_ + "/fmu/takeoff_status/out",
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::takeoff_status_callback,
      this,
      std::placeholders::_1),
    takeoff_status_opts);

  // vehicle_command_ack
  auto vehicle_command_ack_opts = rclcpp::SubscriptionOptions();
  vehicle_command_ack_opts.callback_group = vehicle_command_ack_cgroup_;
  vehicle_command_ack_sub_ = this->create_subscription<VehicleCommandAck>(
    agent_node_name_ + "/fmu/vehicle_command_ack/out",
    DUAQoS::get_datum_qos(),
    std::bind(
      &FlightControlNode::vehicle_command_ack_callback,
      this,
      std::placeholders::_1),
    vehicle_command_ack_opts);
}

/**
 * @brief Routine to initialize TF2 entities.
 */
void FlightControlNode::init_tf2()
{
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Initialize local data
  map_frame_ = "map";
  odom_frame_ = link_namespace_ + "odom";
}

/**
 * @brief Routine to initialize topic publishers.
 */
void FlightControlNode::init_publishers()
{
  // ekf2_odometry_
  ekf2_odometry_pub_ = this->create_publisher<Odometry>(
    "~/ekf2_odometry",
    DUAQoS::get_datum_qos());

  // rviz_ekf2_odometry
  rviz_ekf2_odometry_pub_ = this->create_publisher<Odometry>(
    "~/rviz/ekf2_odometry",
    DUAQoS::Visualization::get_datum_qos());

  // offboard_control_mode
  offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
    agent_node_name_ + "/fmu/offboard_control_mode/in",
    DUAQoS::get_datum_qos());

  // pose
  pose_pub_ = this->create_publisher<EulerPoseStamped>(
    "~/ekf2_pose",
    DUAQoS::get_datum_qos());

  // RViz pose
  rviz_pose_pub_ = this->create_publisher<PoseStamped>(
    "~/rviz/ekf2_pose",
    DUAQoS::Visualization::get_datum_qos());

  // RViz position setpoint
  rviz_position_setpoint_pub_ = this->create_publisher<PoseStamped>(
    "~/rviz/position_setpoint",
    DUAQoS::Visualization::get_datum_qos());

  // trajectory_setpoint
  trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
    agent_node_name_ + "/fmu/trajectory_setpoint/in",
    DUAQoS::get_datum_qos());

  // vehicle_command
  vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
    agent_node_name_ + "/fmu/vehicle_command/in",
    DUAQoS::get_datum_qos());

  // vehicle_rates_setpoint
  vehicle_rates_setpoint_pub_ = this->create_publisher<VehicleRatesSetpoint>(
    agent_node_name_ + "/fmu/vehicle_rates_setpoint/in",
    DUAQoS::get_datum_qos());

  // vehicle_visual_odometry
  visual_odometry_pub_ = this->create_publisher<VehicleVisualOdometry>(
    agent_node_name_ + "/fmu/vehicle_visual_odometry/in",
    DUAQoS::get_datum_qos());
}

/**
 * @brief Routine to initialize message filters.
 */
void FlightControlNode::init_msg_filters()
{
  // Initialize subscribers
  local_pos_sub_ = std::make_shared<message_filters::Subscriber<VehicleLocalPositionStamped>>(
    this,
    agent_node_name_ + "/fmu/vehicle_local_position_stamped/out",
    DUAQoS::get_datum_qos().get_rmw_qos_profile());
  attitude_sub_ = std::make_shared<message_filters::Subscriber<VehicleAttitudeStamped>>(
    this,
    agent_node_name_ + "/fmu/vehicle_attitude_stamped/out",
    DUAQoS::get_datum_qos().get_rmw_qos_profile());

  // Initialize synchronizers
  pose_synchronizer_ = std::make_shared<message_filters::Synchronizer<pose_sync_policy>>(
    pose_sync_policy(10),
    *local_pos_sub_,
    *attitude_sub_);
  pose_synchronizer_->registerCallback(
    &FlightControlNode::pose_callback,
    this);
}

/**
 * @brief Routine to initialize services.
 */
void FlightControlNode::init_services()
{
  // px4_reboot
  reboot_server_ = this->create_service<Trigger>(
    "~/px4_reboot",
    std::bind(
      &FlightControlNode::reboot_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    reboot_cgroup_);

  // reset
  reset_server_ = this->create_service<Trigger>(
    "~/reset",
    std::bind(
      &FlightControlNode::reset_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    reset_cgroup_);

  // setpoints_switch
  setpoints_switch_server_ = this->create_service<SetBool>(
    "~/setpoints_switch",
    std::bind(
      &FlightControlNode::setpoints_switch_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    setpoints_switch_cgroup_);
}

/**
 * @brief Routine to initialize actions.
 */
void FlightControlNode::init_actions()
{
  // arm
  arm_server_ = rclcpp_action::create_server<Arm>(
    this,
    "~/arm",
    std::bind(
      &FlightControlNode::handle_arm_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_arm_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_arm_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);

  // disarm
  disarm_server_ = rclcpp_action::create_server<Disarm>(
    this,
    "~/disarm",
    std::bind(
      &FlightControlNode::handle_disarm_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_disarm_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_disarm_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);

  // landing
  landing_server_ = rclcpp_action::create_server<Landing>(
    this,
    "~/landing",
    std::bind(
      &FlightControlNode::handle_landing_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_landing_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_landing_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);

  // reach
  reach_server_ = rclcpp_action::create_server<Reach>(
    this,
    "~/reach",
    std::bind(
      &FlightControlNode::handle_reach_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_reach_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_reach_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);

  // takeoff
  takeoff_server_ = rclcpp_action::create_server<Takeoff>(
    this,
    "~/takeoff",
    std::bind(
      &FlightControlNode::handle_takeoff_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_takeoff_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_takeoff_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);

  // turn
  turn_server_ = rclcpp_action::create_server<Turn>(
    this,
    "~/turn",
    std::bind(
      &FlightControlNode::handle_turn_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FlightControlNode::handle_turn_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FlightControlNode::handle_turn_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);
}

} // namespace flight_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(flight_control::FlightControlNode)
