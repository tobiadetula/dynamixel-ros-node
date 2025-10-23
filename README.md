For setup:
```bash
sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk

```

To run the node use:
```bash
source install/setup.zsh
ros2 run dynamixel_sdk_examples read_write_node
```

To publish setpoints use:
```bash
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 7, position: 101}"

```