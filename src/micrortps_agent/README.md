# micrortps_agent

Communication layer between PX4 and ROS 2. Based on the [`px4_ros_com`](https://github.com/PX4/px4_ros_com/tree/release/1.13) project.

## Abstract

This package offers a communication layer between the PX4 firmware and its uORB middleware, and a ROS 2 network, by (de)serializing data over a FastDDS network. It is essentially a reimplementation of the `px4_ros_com` project, aimed at fully supporting ROS 2 Humble Hawksbill and version 1.13.3 of the PX4 firmware offering a more modular and flexible design, and a more robust and reliable implementation.

## Contents

The package builds the `micrortps_agent` node and the `micrortps_agent_app` executable. It also offers configuration and launch files.

## Usage

First, you must configure the Bridge topics in the PX4 firmware codebase. At the end of the procedure, you should have generated your version of the [`urtps_bridge_topics.yaml`](src/templates/urtps_bridge_topics.yaml) file. This repository offers a default one with a basic set of topics enabled.

This package can be built using `colcon`. Find the dependencies listed in the [`package.xml`](package.xml) file.

Then, you can either launch the node or the executable.

A set of parameters is offered for the `micrortps_agent` node, see the [`params.yaml`](src/micrortps_agent/params.yaml) file for details.

### Default uORB topics list

Currently, this module is kept aligned with our fork of the [`PX4-Autopilot`](https://github.com/dotX-Automation/PX4-Autopilot) project. The list of uORB topics that are (de)serialized by default follows. Additionally, some debug and alias topics have been left active for compatibility with PX4. Communication directions are expressed in a PX4-centric logic.

#### Basic topics

These messages are necessary to perform any flight operation.

| Topic Name | IN/OUT | Description |
|    :---:   | :---: |    :---:    |
| battery_status | out | Battery status information |
| offboard_control_mode | in | Used to switch to OFFBOARD mode |
| takeoff_status | out | Drone airborne status changes |
| timesync | both | Clock synchronization |
| timesync_status | out | Clock synchronization status |
| trajectory_setpoint | in | (X, Y, Z, yaw) setpoints |
| vehicle_attitude | out | Current attitude quaternion to local NED frame from EKF2 |
| vehicle_command | in | Flight operations commands |
| vehicle_command_ack | out | Command feedbacks |
| vehicle_local_position | out | Current position in local NED frame from EKF2 |
| vehicle_visual_odometry | in | Pose samples from external VIO system |

#### Optional topics

These messages are not required to fly but may be useful for debugging and testing.

| Topic Name | IN/OUT | Description |
|    :---:   | :---: |    :---:    |
| estimator_status_flags | out | EKF2 internal status information |
| log_message | out | PX4 internal log messages |
| sensor_combined | out | Real-time samples from all IMU sensors |
| vehicle_odometry | out | VIO data parsing results |

#### Additional node topics

These messages are published directly by a ROS 2 node managed by the Agent application. Their data comes from messages sent by PX4 and usually holds additional information (*e.g.*, a ROS 2 `std_msgs/Header` with a timestamp).

| Topic Name | IN/OUT | Description |
|    :---:   | :---: |    :---:    |
| vehicle_attitude_stamped | out | Current attitude quaternion to local frame from EKF2 with ROS 2 timestamp |
| vehicle_local_position_stamped | out | Current position in local frame from EKF2 with ROS 2 timestamp |

### Frame conventions

PX4 internally uses a NED frame convention, *i.e.*, with the Z axis pointing downwards. ROS 2, as well as many other software, prefers to express data using an NWU convention, *i.e.*, with the Z axis pointing upwards. This package publishes data in both ways, meaning:

- Raw data that mirrors PX4 internal uORB topics is left untouched, *i.e.*, in NED.
- Data that is generated, or converted in common formats by the Agent node is converted to NWU.

---

## License

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

## Copyright

Copyright (c) 2024, dotX Automation s.r.l.

Copyright (c) 2017, Proyectos y Sistemas de Mantenimiento SL (eProsima)

Copyright (c) 2018-2021, PX4 Development Team
