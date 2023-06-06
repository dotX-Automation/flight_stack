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

/**
 * @brief Routine to initialize node parameters.
 */
void FlightControlNode::init_parameters()
{
  // Register parameter updates callback
  on_set_params_chandle_ = this->add_on_set_parameters_callback(
    std::bind(
      &FlightControlNode::on_set_parameters_callback,
      this,
      std::placeholders::_1));

  // FMU command transmission attempts
  declare_int_parameter(
    "fmu_command_attempts",
    3, 1, 10, 1,
    "Transmission attempts for FMU commands.",
    "Must be positive, too high is unnecessary.",
    false,
    fmu_command_attempts_descriptor_);

  // FMU command ACK timeout
  declare_int_parameter(
    "fmu_command_timeout",
    5000, 1000, 10000, 1,
    "Timeout for ACKs sent over the vehicle_command_ack topic [ms].",
    "Cannot be zero, just a few seconds should be enough.",
    false,
    fmu_command_timeout_descriptor_);

  // Landing operation timeout
  declare_int_parameter(
    "landing_timeout",
    20000, 1000, 60000, 1,
    "Landing operation timeout [ms].",
    "Cannot be zero, must usually be higher than 10 s.",
    false,
    landing_timeout_descriptor_);

  // Low battery voltage
  declare_double_parameter(
    "low_battery_voltage",
    21.0, 20.0, 25.0, 0.5,
    "Low battery voltage level [V].",
    "Only for testing.",
    true,
    low_battery_voltage_descriptor_);

  // Battery monitoring flag
  declare_bool_parameter(
    "monitor_battery",
    false,
    "Activates subscription to battery_status topic.",
    "Only for testing.",
    true,
    monitor_battery_descriptor_);

  // Roll-pitch stabilization confidence
  declare_double_parameter(
    "roll_pitch_stabilization_confidence",
    0.087266, 0.0174533, 1.570796, 0.0,
    "Roll-pitch stabilization confidence interval radius [rad].",
    "Must not be zero or higher than 90°.",
    false,
    roll_pitch_stabilization_confidence_descriptor_);

  // Setpoints publishing timer period
  declare_int_parameter(
    "setpoints_period",
    50, 1, 500, 1,
    "Setpoints publishing interval [ms].",
    "Frequency must not be lower than 2 Hz.",
    false,
    setpoints_period_descriptor_);

  // Takeoff position confidence
  declare_double_parameter(
    "takeoff_position_confidence",
    0.1, 0.001, 1.0, 0.001,
    "Takeoff position confidence sphere radius [m].",
    "Must not be zero, too high is unnecessary, three digits.",
    false,
    takeoff_position_confidence_descriptor_);

  // Takeoff operation timeout
  declare_int_parameter(
    "takeoff_timeout",
    15000, 1000, 60000, 1,
    "Takeoff operation timeout [ms].",
    "Cannot be zero, must usually be higher than 5 s.",
    false,
    takeoff_timeout_descriptor_);

  // Travel sleep time
  declare_int_parameter(
    "travel_sleep_time",
    200, 1, 5000, 1,
    "Reach and takeoff threads polling period [ms].",
    "Must not be too low in order not to waste CPU cycles.",
    false,
    travel_sleep_time_descriptor_);

  // Turn sleep time
  declare_int_parameter(
    "turn_sleep_time",
    100, 1, 500, 1,
    "Turn threads polling period [ms].",
    "Must not be too low in order not to waste CPU cycles.",
    false,
    turn_sleep_time_descriptor_);

  // Turn step
  declare_double_parameter(
    "turn_step",
    0.261799, 0.0174533, 0.785398, 0.0,
    "Yaw step used to discretize turns in smaller legs.",
    "Must not be zero nor higher than 45°.",
    false,
    turn_step_descriptor_);

  // Max horizontal velocity for stabilization
  declare_double_parameter(
    "v_horz_stabilization_max",
    0.1, 0.01, 1.0, 0.01,
    "Maximum horizontal velocity for stabilization [m/s].",
    "Must not be zero, too high is unnecessary, two digits.",
    false,
    v_horz_stabilization_max_descriptor_);

  // Max vertical velocity for stabilization
  declare_double_parameter(
    "v_vert_stabilization_max",
    0.1, 0.01, 1.0, 0.01,
    "Maximum vertical velocity for stabilization [m/s].",
    "Must not be zero, too high is unnecessary, two digits.",
    false,
    v_vert_stabilization_max_descriptor_);

  // Yaw stabilization confidence
  declare_double_parameter(
    "yaw_stabilization_confidence",
    0.261799, 0.0174533, 1.570796, 0.0,
    "Yaw stabilization confidence interval radius [rad].",
    "Must not be zero or higher than 90°, six digits.",
    false,
    yaw_confidence_descriptor_);
}

/**
 * @brief Parameters update validation callback.
 *
 * @param params Vector of parameters for which a change has been requested.
 * @return Operation result in SetParametersResult message.
 */
SetParametersResult FlightControlNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // Acquire the main operations lock: can't change parameters while some
  // flight operation might require them
  if (pthread_spin_trylock(&(this->operation_lock_))) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Flight operation in progress, parameter update denied");
    res.set__reason("Flight operation in progress");
    return res;
  }

  // First, check if each update is feasible
  // Initial checks must be added here!
  for (const rclcpp::Parameter & p : params) {
    // FMU command transmission attempts
    if (p.get_name() == "fmu_command_attempts") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for fmu_command_attempts");
        break;
      }
      continue;
    }

    // FMU command ACK timeout
    if (p.get_name() == "fmu_command_timeout") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for fmu_command_timeout");
        break;
      }
      continue;
    }

    // Landing operation timeout
    if (p.get_name() == "landing_timeout") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for landing_timeout");
        break;
      }
      continue;
    }

    // Low battery voltage
    if (p.get_name() == "low_battery_voltage") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for low_battery_voltage");
        break;
      }
      continue;
    }

    // Battery monitoring flag
    if (p.get_name() == "monitor_battery") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for monitor_battery");
        break;
      }
      continue;
    }

    // Roll-pitch stabilization confidence
    if (p.get_name() == "roll_pitch_stabilization_confidence") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for roll_pitch_stabilization_confidence");
        break;
      }
      continue;
    }

    // Setpoints publishing timer period
    if (p.get_name() == "setpoints_period") {
      // Check type and if the drone is disarmed
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for setpoints_period");
        break;
      }
      if (armed_.load(std::memory_order_acquire)) {
        res.set__successful(false);
        res.set__reason("Drone is ARMED, cannot change setpoints_period now");
        break;
      }
      continue;
    }

    // Takeoff position confidence
    if (p.get_name() == "takeoff_position_confidence") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for takeoff_position_confidence");
        break;
      }
      continue;
    }

    // Takeoff operation timeout
    if (p.get_name() == "takeoff_timeout") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for takeoff_timeout");
        break;
      }
      continue;
    }

    // Travel sleep time
    if (p.get_name() == "travel_sleep_time") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for travel_sleep_time");
        break;
      }
      continue;
    }

    // Turn sleep time
    if (p.get_name() == "turn_sleep_time") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for turn_sleep_time");
        break;
      }
      continue;
    }

    // Turn step
    if (p.get_name() == "turn_step") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for turn_step");
        break;
      }
      continue;
    }

    // Max horizontal velocity for stabilization
    if (p.get_name() == "v_horz_stabilization_max") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for v_horz_stabilization_max");
        break;
      }
      continue;
    }

    // Max vertical velocity for stabilization
    if (p.get_name() == "v_vert_stabilization_max") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for v_vert_stabilization_max");
        break;
      }
      continue;
    }

    // Yaw stabilization confidence
    if (p.get_name() == "yaw_stabilization_confidence") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for yaw_stabilization_confidence");
        break;
      }
      continue;
    }
  }
  if (!res.successful) {
    pthread_spin_unlock(&(this->operation_lock_));
    return res;
  }

  // Then, do what is necessary to update each parameter
  // Add ad-hoc update procedures must be added here!
  for (const rclcpp::Parameter & p : params) {
    // FMU command transmission attempts
    if (p.get_name() == "fmu_command_attempts") {
      fmu_command_attempts_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "fmu_command_attempts: %ld",
        fmu_command_attempts_);
      continue;
    }

    // FMU command ACK timeout
    if (p.get_name() == "fmu_command_timeout") {
      fmu_command_timeout_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "fmu_command_timeout: %ld ms",
        fmu_command_timeout_);
      continue;
    }

    // Landing operation timeout
    if (p.get_name() == "landing_timeout") {
      landing_timeout_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "landing_timeout: %ld ms",
        landing_timeout_);
      continue;
    }

    // Low battery voltage
    if (p.get_name() == "low_battery_voltage") {
      low_battery_voltage_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "low_battery_voltage: %f V",
        low_battery_voltage_);
      continue;
    }

    // Battery monitoring flag
    if (p.get_name() == "monitor_battery") {
      monitor_battery_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "monitor_battery: %s",
        monitor_battery_ ? "true" : "false");
      continue;
    }

    // Roll-pitch stabilization confidence
    if (p.get_name() == "roll_pitch_stabilization_confidence") {
      roll_pitch_stabilization_confidence_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "roll_pitch_stabilization_confidence: %f rad (%f°)",
        roll_pitch_stabilization_confidence_,
        roll_pitch_stabilization_confidence_ * 180.0f / M_PIf32);
      continue;
    }

    // Setpoint publishing timer period
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
    }

    // Takeoff position confidence
    if (p.get_name() == "takeoff_position_confidence") {
      takeoff_position_confidence_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "takeoff_position_confidence: %f m",
        takeoff_position_confidence_);
      continue;
    }

    // Takeoff operation timeout
    if (p.get_name() == "takeoff_timeout") {
      takeoff_timeout_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "takeoff_timeout: %ld ms",
        takeoff_timeout_);
      continue;
    }

    // Travel sleep time
    if (p.get_name() == "travel_sleep_time") {
      travel_sleep_time_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "travel_sleep_time: %ld ms",
        travel_sleep_time_);
      continue;
    }

    // Turn sleep time
    if (p.get_name() == "turn_sleep_time") {
      turn_sleep_time_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "turn_sleep_time: %ld ms",
        turn_sleep_time_);
      continue;
    }

    // Turn step
    if (p.get_name() == "turn_step") {
      turn_step_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "turn_step: %f rad (%f°)",
        turn_step_,
        turn_step_ * 180.0f / M_PIf32);
      continue;
    }

    // Max horizontal velocity for stabilization
    if (p.get_name() == "v_horz_stabilization_max") {
      v_horz_stabilization_max_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "v_horz_stabilization_max: %f m/s",
        v_horz_stabilization_max_);
      continue;
    }

    // Max vertical velocity for stabilization
    if (p.get_name() == "v_vert_stabilization_max") {
      v_vert_stabilization_max_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "v_vert_stabilization_max: %f m/s",
        v_vert_stabilization_max_);
      continue;
    }

    // Yaw stabilization confidence
    if (p.get_name() == "yaw_stabilization_confidence") {
      yaw_stabilization_confidence_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "yaw_stabilization_confidence: %f rad (%f°)",
        yaw_stabilization_confidence_,
        yaw_stabilization_confidence_ * 180.0f / M_PIf32);
      continue;
    }
  }

  pthread_spin_unlock(&(this->operation_lock_));
  return res;
}

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
