# Dynamixel ROS Node - Detailed Usage Guide

## Table of Contents
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Examples](#examples)
- [Advanced Usage](#advanced-usage)
- [API Reference](#api-reference)

## Getting Started

### First Time Setup

1. **Install dependencies:**
```bash
pip3 install -r requirements.txt
```

2. **Set up serial port permissions:**
```bash
# Option 1: Temporary (needs to be done after each reboot)
sudo chmod 666 /dev/ttyUSB0

# Option 2: Permanent (recommended)
sudo usermod -aG dialout $USER
# Log out and log back in for changes to take effect
```

3. **Test the connection:**
```bash
# Check if the device is detected
ls -l /dev/ttyUSB*

# Launch with verbose output
roslaunch dynamixel_ros_node dynamixel_controller.launch
```

## Configuration

### Motor ID Configuration

Before using, ensure your motors have the correct IDs set. You can use the Dynamixel Wizard software to configure motor IDs.

### Baudrate Settings

Common baudrate values:
- 57600 (default for most motors)
- 115200
- 1000000 (1Mbps - recommended for multiple motors)
- 2000000 (2Mbps)

### Protocol Version

- Protocol 1.0: Older Dynamixel models (AX, RX, MX series with firmware < v40)
- Protocol 2.0: Newer models (XM, XH, PRO series, MX with firmware >= v40)

## Examples

### Example 1: Single Motor Control

```bash
# Terminal 1: Launch controller
roslaunch dynamixel_ros_node dynamixel_controller.launch

# Terminal 2: Send commands
rostopic pub /motor_command std_msgs/Float64 "data: 2048.0"
```

### Example 2: Multiple Motors

```bash
# Launch with multiple motor IDs
roslaunch dynamixel_ros_node multi_motor.launch motor_ids:="[1,2,3]"
```

### Example 3: Using Python Interface

Create a file `my_control.py`:
```python
#!/usr/bin/env python3
import rospy
from motor_interface import MotorInterface

def main():
    rospy.init_node('my_motor_control')
    interface = MotorInterface()
    
    # Move to home position
    interface.move_to_position(0)
    rospy.sleep(2)
    
    # Move to center
    interface.move_to_position(2048)
    rospy.sleep(2)
    
    # Move to max
    interface.move_to_position(4095)

if __name__ == '__main__':
    main()
```

Run it:
```bash
chmod +x my_control.py
rosrun dynamixel_ros_node my_control.py
```

### Example 4: Running Test Sequence

```bash
# Make sure controller is running first
roslaunch dynamixel_ros_node dynamixel_controller.launch

# In another terminal, run test
rosrun dynamixel_ros_node simple_motor_test.py
```

## Advanced Usage

### Custom Control Table Addresses

If using different Dynamixel models, you may need to adjust control table addresses in `dynamixel_controller.py`:

```python
# For Protocol 1.0 (e.g., AX-12)
self.ADDR_TORQUE_ENABLE = 24
self.ADDR_GOAL_POSITION = 30
self.ADDR_PRESENT_POSITION = 36

# For Protocol 2.0 (e.g., XM430)
self.ADDR_TORQUE_ENABLE = 64
self.ADDR_GOAL_POSITION = 116
self.ADDR_PRESENT_POSITION = 132
```

### Monitoring Performance

```bash
# Monitor joint states update rate
rostopic hz /joint_states

# Monitor command latency
rostopic delay /motor_command

# View all active topics
rostopic list
```

### Integration with MoveIt!

This node publishes to `/joint_states` which can be used with MoveIt! for motion planning:

1. Create a URDF/XACRO model of your robot
2. Generate MoveIt! configuration
3. The joint states will automatically be used by MoveIt!

### Using with ROS Services (Future Enhancement)

For more advanced control, consider adding ROS services for:
- Setting velocity limits
- Configuring PID gains
- Reading motor temperature
- Setting torque limits

## API Reference

### Topics

#### Published
- `/joint_states` (sensor_msgs/JointState)
  - Current position, velocity of all motors
  - Published at `update_rate` frequency
  
- `/motor_status` (std_msgs/Bool)
  - Controller operational status

#### Subscribed
- `/motor_command` (std_msgs/Float64)
  - Position command for all motors
  - Value range: 0-4095 (for most models)

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| device_name | string | /dev/ttyUSB0 | Serial port device |
| baudrate | int | 57600 | Communication baudrate |
| protocol_version | float | 2.0 | Dynamixel protocol (1.0 or 2.0) |
| motor_ids | list | [1] | List of motor IDs to control |
| update_rate | int | 50 | Joint state publishing rate (Hz) |

### Utility Functions

From `dynamixel_utils.py`:

```python
# Position conversions
position_to_radians(position, resolution=4096)
radians_to_position(radians, resolution=4096)
position_to_degrees(position, resolution=4096)
degrees_to_position(degrees, resolution=4096)

# Value clamping
clamp(value, min_value, max_value)
```

## Troubleshooting

### Issue: "Failed to open port"
- Check device name is correct
- Verify permissions (run `ls -l /dev/ttyUSB0`)
- Ensure no other program is using the port

### Issue: "Failed to enable torque"
- Check motor ID is correct
- Verify motor is powered
- Ensure baudrate matches motor configuration

### Issue: Position not changing
- Verify motor mode is set to position control (not velocity or extended position)
- Check torque is enabled
- Verify position value is within valid range (0-4095)

### Issue: Slow response
- Increase baudrate if possible
- Reduce update_rate
- Check for loose connections

## Tips and Best Practices

1. **Start with low baudrate** (57600) for initial testing, increase for production
2. **Always disable torque** before manually moving motors
3. **Monitor motor temperature** during extended use
4. **Use sync_write** for controlling multiple motors simultaneously (future enhancement)
5. **Keep cables short** and use quality USB cables
6. **Power motors separately** from control board for better stability

## Further Resources

- [Dynamixel SDK Documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Dynamixel e-Manual](https://emanual.robotis.com/)
- [ROS Wiki](http://wiki.ros.org/)
