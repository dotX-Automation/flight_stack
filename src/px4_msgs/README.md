# px4_msgs

Message definitions for the ROS 2 side of the microRTPS bridge.

## Abstract

This package contains the ROS 2 message definitions of those included in the [PX4 firmware](https://px4.io/). Building this package generates all the required interfaces to interface ROS 2 nodes with the PX4 Autopilot internals, which use the [uORB messaging API](https://dev.px4.io/en/middleware/uorb.html).

**This package is currently aligned to version 1.13.3 of the PX4 firmware, and is intended to work with the microRTPS bridge only.**

## Usage

Just build this like a normal ROS 2 package, and the generated message definitions will be available in your workspace.

### How are these messsage definitions generated?

The ROS message definitions can be generated with a Python script found in the PX4 repo, generally named `uorb_to_ros_msgs.py`. One can also use this script to generate its own ROS message definitions for new or modified uORB messages. This step is necessary since the definitions in this package must match the uORB ones that the firmware uses.

---

## License

This work is licensed under the BSD 3-Clause License. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2012 - 2022, PX4 Development Team

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
