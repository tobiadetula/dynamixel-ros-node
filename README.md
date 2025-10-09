# Dynamixel ROS Node

ROS package for controlling Dynamixel motors using the Dynamixel SDK.

## Overview

This package provides a ROS interface for controlling Dynamixel servo motors. It supports multiple motors, position control, and real-time joint state publishing.

## Features

- Control single or multiple Dynamixel motors
- Position control via ROS topics
- Real-time joint state publishing
- Support for Protocol 1.0 and 2.0
- Configurable via launch files and YAML configs
- Easy-to-use Python interface

## Prerequisites

- ROS (Melodic, Noetic, or newer)
- Python 3
- Dynamixel SDK

### Install Dynamixel SDK

```bash
sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk
# OR via pip
pip3 install dynamixel-sdk
```

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/tobiadetula/dynamixel-ros-node.git
```

2. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
# OR if using catkin tools
catkin build dynamixel_ros_node
```

3. Source the workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

## Hardware Setup

1. Connect your Dynamixel motor(s) to your computer via USB2Dynamixel or U2D2
2. Check the device port (usually `/dev/ttyUSB0` on Linux)
3. Ensure you have read/write permissions:
```bash
sudo chmod 666 /dev/ttyUSB0
# OR add your user to dialout group
sudo usermod -aG dialout $USER
```

## Usage

### Basic Usage (Single Motor)

Launch the controller with default settings:
```bash
roslaunch dynamixel_ros_node dynamixel_controller.launch
```

### Multi-Motor Setup

For controlling multiple motors:
```bash
roslaunch dynamixel_ros_node multi_motor.launch motor_ids:="[1,2,3,4]"
```

### Custom Configuration

Launch with custom parameters:
```bash
roslaunch dynamixel_ros_node dynamixel_controller.launch \
  device_name:=/dev/ttyUSB0 \
  baudrate:=1000000 \
  motor_ids:="[1,2]" \
  update_rate:=100
```

### Sending Commands

Send position commands via ROS topics:
```bash
# Move motor to position 2048
rostopic pub /motor_command std_msgs/Float64 "data: 2048.0"
```

### Monitor Joint States

View current motor positions:
```bash
rostopic echo /joint_states
```

## Python Interface

Use the provided Python interface for programmatic control:

```python
#!/usr/bin/env python3
import rospy
from motor_interface import MotorInterface

rospy.init_node('my_motor_control')
interface = MotorInterface()

# Move to specific position
interface.move_to_position(2048)

# Move through sequence
positions = [0, 1024, 2048, 3072, 4096]
interface.move_sequence(positions, delay=2.0)
```

## Configuration Files

### motor_config.yaml

Configure motor parameters, IDs, and connection settings.

### motor_limits.yaml

Define position and velocity limits for each motor.

## Topics

### Published Topics

- `/joint_states` (sensor_msgs/JointState): Current state of all motors
- `/motor_status` (std_msgs/Bool): Controller status

### Subscribed Topics

- `/motor_command` (std_msgs/Float64): Position commands for motors

## Parameters

- `~device_name` (string, default: "/dev/ttyUSB0"): Serial port device
- `~baudrate` (int, default: 57600): Communication baudrate
- `~protocol_version` (float, default: 2.0): Dynamixel protocol version
- `~motor_ids` (list, default: [1]): List of motor IDs to control
- `~update_rate` (int, default: 50): Joint state publishing rate (Hz)

## Troubleshooting

### Port Access Issues
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Motor Not Responding
- Check motor ID configuration
- Verify baudrate matches motor settings
- Ensure proper power supply to motors
- Check cable connections

### Communication Errors
- Try reducing baudrate
- Check for loose connections
- Verify protocol version matches motor firmware

## Development

### File Structure
```
dynamixel_ros_node/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   ├── dynamixel_controller.py
│   └── motor_interface.py
├── launch/
│   ├── dynamixel_controller.launch
│   └── multi_motor.launch
├── config/
│   ├── motor_config.yaml
│   └── motor_limits.yaml
└── src/
    ├── __init__.py
    └── dynamixel_utils.py
```

## License

MIT License

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Author

Tobia Detula

## Acknowledgments

- Dynamixel SDK by ROBOTIS
- ROS Community
