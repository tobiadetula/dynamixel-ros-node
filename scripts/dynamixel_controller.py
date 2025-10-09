#!/usr/bin/env python3
"""
Dynamixel Motor Controller Node

This ROS node provides an interface to control Dynamixel motors.
It subscribes to command topics and publishes motor status.
"""

import rospy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
import yaml
import os


class DynamixelController:
    """Main controller class for Dynamixel motors"""
    
    def __init__(self):
        """Initialize the Dynamixel controller"""
        rospy.init_node('dynamixel_controller', anonymous=False)
        
        # Load parameters
        self.device_name = rospy.get_param('~device_name', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 57600)
        self.protocol_version = rospy.get_param('~protocol_version', 2.0)
        
        # Motor IDs
        self.motor_ids = rospy.get_param('~motor_ids', [1])
        
        # Control table addresses (for Protocol 2.0)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_MOVING = 122
        self.ADDR_PRESENT_VELOCITY = 128
        
        # Initialize PortHandler and PacketHandler
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)
        
        # Open port
        if not self.port_handler.openPort():
            rospy.logerr("Failed to open port")
            return
        
        # Set baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr("Failed to set baudrate")
            return
        
        rospy.loginfo("Dynamixel controller initialized successfully")
        
        # Enable torque for all motors
        for motor_id in self.motor_ids:
            self.enable_torque(motor_id)
        
        # Publishers
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.status_pub = rospy.Publisher('motor_status', Bool, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('motor_command', Float64, self.command_callback)
        
        # Update rate
        self.rate = rospy.Rate(rospy.get_param('~update_rate', 50))
        
    def enable_torque(self, motor_id):
        """Enable torque for a motor"""
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(f"Failed to enable torque for motor {motor_id}")
        else:
            rospy.loginfo(f"Torque enabled for motor {motor_id}")
    
    def disable_torque(self, motor_id):
        """Disable torque for a motor"""
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(f"Failed to disable torque for motor {motor_id}")
    
    def set_goal_position(self, motor_id, position):
        """Set goal position for a motor"""
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_GOAL_POSITION, int(position))
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(f"Failed to set position for motor {motor_id}")
    
    def read_position(self, motor_id):
        """Read current position of a motor"""
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS:
            return None
        
        return dxl_present_position
    
    def command_callback(self, msg):
        """Callback for motor command messages"""
        position = msg.data
        for motor_id in self.motor_ids:
            self.set_goal_position(motor_id, position)
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        
        for motor_id in self.motor_ids:
            position = self.read_position(motor_id)
            if position is not None:
                joint_state.name.append(f"motor_{motor_id}")
                joint_state.position.append(position)
        
        self.joint_state_pub.publish(joint_state)
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Dynamixel controller running...")
        
        while not rospy.is_shutdown():
            self.publish_joint_states()
            self.rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down Dynamixel controller...")
        
        # Disable torque for all motors
        for motor_id in self.motor_ids:
            self.disable_torque(motor_id)
        
        # Close port
        self.port_handler.closePort()


if __name__ == '__main__':
    try:
        controller = DynamixelController()
        rospy.on_shutdown(controller.shutdown)
        controller.run()
    except rospy.ROSInterruptException:
        pass
