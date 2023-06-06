/**
 * Flight Control node initialization and parameters routines.
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

#include <flight_control/flight_control.hpp>

namespace FlightControl
{

/**
 * @brief Flight Control node constructor.
 *
 * @param node_opts Options for the base node.
 */
FlightControlNode::FlightControlNode(const rclcpp::NodeOptions & node_options)
: Node("flight_control", node_options)
{
  // Initialize atomic members
  init_atomics();

  // Initialize callback groups
  init_cgroups();

  // Initialize synchronization primitives
  init_sync_primitives();

  // Initialize node parameters
  init_parameters();

  // Initialize topic subscriptions
  init_subscriptions();

  // Initialize topic publishers
  init_publishers();

  // Initialize message filters
  init_msg_filters();

  // Initialize services
  init_services();

  // Initialize actions
  init_actions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Flight Control node destructor.
 */
FlightControlNode::~FlightControlNode()
{
  // Destroy the synchronization primitives (prevents memory leaks)
  pthread_spin_destroy(&(this->state_lock_));
  pthread_spin_destroy(&(this->setpoint_lock_));
  pthread_spin_destroy(&(this->operation_lock_));
}

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
  fmu_timestamp_.store(0UL, std::memory_order_release);
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
  px4_timestamp_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  position_setpoint_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  velocity_setpoint_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  takeoff_status_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  vehicle_command_ack_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Services
  reset_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  setpoints_switch_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Actions
  actions_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize synchronization primitives used internally.
 */
void FlightControlNode::init_sync_primitives()
{
  create_spinlock(&(this->state_lock_));
  create_spinlock(&(this->setpoint_lock_));
  create_spinlock(&(this->operation_lock_));
}

    /*// Setpoint publishing timer period
    // TODO
    if (p.get_name() == "setpoints_period") {
      setpoints_period_ = p.as_int();
      // If timer is up, reset it too
      if (setpoints_timer_ != nullptr) {
        deactivate_setpoints_timer();
        activate_setpoints_timer();
        RCLCPP_INFO(
          this->get_logger(),
          "setpoints_period: %ld ms, timer reset",
          setpoints_period_);
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "setpoints_period: %ld ms",
          setpoints_period_);
      }
      continue;
    }*/

/**
 * @brief Routine to initialize topic subscriptions.
 */
void FlightControlNode::init_subscriptions()
{
  // battery_status (configurable)
  if (this->get_parameter("monitor_battery").as_bool()) {
    auto battery_status_opts = rclcpp::SubscriptionOptions();
    battery_status_opts.callback_group = battery_state_cgroup_;
    battery_state_sub_ = this->create_subscription<BatteryStatus>(
      "/fmu/battery_status/out",
      rclcpp::QoS(1),
      std::bind(
        &FlightControlNode::battery_state_callback,
        this,
        std::placeholders::_1),
      battery_status_opts);
  }

  // log_message
  auto log_message_opts = rclcpp::SubscriptionOptions();
  log_message_opts.callback_group = log_message_cgroup_;
  log_message_sub_ = this->create_subscription<LogMessage>(
    "/fmu/log_message/out",
    rclcpp::QoS(1),
    std::bind(
      &FlightControlNode::log_message_callback,
      this,
      std::placeholders::_1),
    log_message_opts);

  // px4_timestamp
  auto px4_timestamp_opts = rclcpp::SubscriptionOptions();
  px4_timestamp_opts.callback_group = px4_timestamp_cgroup_;
  px4_timestamp_sub_ = this->create_subscription<PX4Timestamp>(
    "/fmu/micrortps_agent_node/px4_timestamp/out",
    rclcpp::QoS(1),
    std::bind(
      &FlightControlNode::px4_timestamp_callback,
      this,
      std::placeholders::_1),
    px4_timestamp_opts);

  // position_setpoint
  auto position_setpoint_opts = rclcpp::SubscriptionOptions();
  position_setpoint_opts.callback_group = position_setpoint_cgroup_;
  position_setpoint_sub_ = this->create_subscription<PositionSetpoint>(
    "~/position_setpoint",
    rclcpp::QoS(1),
    std::bind(
      &FlightControlNode::position_setpoint_callback,
      this,
      std::placeholders::_1),
    position_setpoint_opts);

  // velocity_setpoint
  auto velocity_setpoint_opts = rclcpp::SubscriptionOptions();
  velocity_setpoint_opts.callback_group = velocity_setpoint_cgroup_;
  velocity_setpoint_sub_ = this->create_subscription<VelocitySetpoint>(
    "~/velocity_setpoint",
    rclcpp::QoS(1),
    std::bind(
      &FlightControlNode::velocity_setpoint_callback,
      this,
      std::placeholders::_1),
    velocity_setpoint_opts);

  // takeoff_status
  auto takeoff_status_opts = rclcpp::SubscriptionOptions();
  takeoff_status_opts.callback_group = takeoff_status_cgroup_;
  takeoff_status_sub_ = this->create_subscription<TakeoffStatus>(
    "/fmu/takeoff_status/out",
    rclcpp::QoS(10),
    std::bind(
      &FlightControlNode::takeoff_status_callback,
      this,
      std::placeholders::_1),
    takeoff_status_opts);

  // vehicle_command_ack
  auto vehicle_command_ack_opts = rclcpp::SubscriptionOptions();
  vehicle_command_ack_opts.callback_group = vehicle_command_ack_cgroup_;
  vehicle_command_ack_sub_ = this->create_subscription<VehicleCommandAck>(
    "/fmu/vehicle_command_ack/out",
    rclcpp::QoS(10),
    std::bind(
      &FlightControlNode::vehicle_command_ack_callback,
      this,
      std::placeholders::_1),
    vehicle_command_ack_opts);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void FlightControlNode::init_publishers()
{
  // offboard_control_mode
  offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
    "/fmu/offboard_control_mode/in",
    rclcpp::QoS(1));

  // pose
  pose_pub_ = this->create_publisher<Pose>(
    "~/pose",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(fc_pose_qos_profile)));

  // RViz pose
  rviz_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/rviz/pose",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(fc_rviz_pose_qos_profile)));

  // trajectory_setpoint
  trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
    "/fmu/trajectory_setpoint/in",
    rclcpp::QoS(1));

  // vehicle_command
  vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
    "/fmu/vehicle_command/in",
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize message filters.
 */
void FlightControlNode::init_msg_filters()
{
  // Initialize subscribers
  local_pos_sub_ = std::make_shared<message_filters::Subscriber<VehicleLocalPositionStamped>>(
    this,
    "/fmu/micrortps_agent_node/vehicle_local_position_stamped/out",
    rmw_qos_profile_sensor_data);
  attitude_sub_ = std::make_shared<message_filters::Subscriber<VehicleAttitudeStamped>>(
    this,
    "/fmu/micrortps_agent_node/vehicle_attitude_stamped/out",
    rmw_qos_profile_sensor_data);

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
  // reset
  reset_server_ = this->create_service<Reset>(
    "~/reset",
    std::bind(
      &FlightControlNode::reset_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    reset_cgroup_);

  // setpoints_switch
  setpoints_switch_server_ = this->create_service<SetpointsSwitch>(
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
  arm_server_options_.allocator = rcl_get_default_allocator();
  arm_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  arm_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  arm_server_options_.result_service_qos = rmw_qos_profile_services_default;
  arm_server_options_.status_topic_qos = fc_action_status_qos_profile;
  arm_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    arm_server_options_,
    actions_cgroup_);

  // disarm
  disarm_server_options_.allocator = rcl_get_default_allocator();
  disarm_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  disarm_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  disarm_server_options_.result_service_qos = rmw_qos_profile_services_default;
  disarm_server_options_.status_topic_qos = fc_action_status_qos_profile;
  disarm_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    disarm_server_options_,
    actions_cgroup_);

  // landing
  landing_server_options_.allocator = rcl_get_default_allocator();
  landing_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  landing_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  landing_server_options_.result_service_qos = rmw_qos_profile_services_default;
  landing_server_options_.status_topic_qos = fc_action_status_qos_profile;
  landing_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    landing_server_options_,
    actions_cgroup_);

  // reach
  reach_server_options_.allocator = rcl_get_default_allocator();
  reach_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  reach_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  reach_server_options_.result_service_qos = rmw_qos_profile_services_default;
  reach_server_options_.status_topic_qos = fc_action_status_qos_profile;
  reach_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    reach_server_options_,
    actions_cgroup_);

  // takeoff
  takeoff_server_options_.allocator = rcl_get_default_allocator();
  takeoff_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  takeoff_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  takeoff_server_options_.result_service_qos = rmw_qos_profile_services_default;
  takeoff_server_options_.status_topic_qos = fc_action_status_qos_profile;
  takeoff_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    takeoff_server_options_,
    actions_cgroup_);

  // turn
  turn_server_options_.allocator = rcl_get_default_allocator();
  turn_server_options_.goal_service_qos = rmw_qos_profile_services_default;
  turn_server_options_.cancel_service_qos = rmw_qos_profile_services_default;
  turn_server_options_.result_service_qos = rmw_qos_profile_services_default;
  turn_server_options_.status_topic_qos = fc_action_status_qos_profile;
  turn_server_options_.feedback_topic_qos = fc_action_feedback_qos_profile;
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
    turn_server_options_,
    actions_cgroup_);
}

} // namespace FlightControl
