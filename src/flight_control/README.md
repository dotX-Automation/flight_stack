# flight_control

High-level drone control software package.

## Abstract

This package provides an abstraction layer towards the PX4 firmware, and thus, the drone that it is controlling.

It creates a set of ROS 2 communication interfaces that allow the user to control the drone in a high-level manner, and to receive information about its state.

It is meant to be used in conjunction with the [px4_msgs](../px4_msgs/README.md) package, which provides the necessary message definitions to interface with the PX4 firmware, and with the microRTPS Agent contained in the [micrortps_agent](../micrortps_agent/README.md) package, which allows the user to communicate with the PX4 firmware via the microRTPS Bridge.

## Contents

TODO.

## Usage

TODO.

### Topics

TODO.

### Services

TODO.

### Actions

TODO.

### Parameters

TODO.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
