#!/usr/bin/env python3
"""
Motor Interface Module

Provides a simplified interface for controlling Dynamixel motors.
"""

import rospy
from std_msgs.msg import Float64


class MotorInterface:
    """Simple interface class for sending commands to motors"""
    
    def __init__(self):
        """Initialize the motor interface"""
        self.command_pub = rospy.Publisher('motor_command', Float64, queue_size=10)
        rospy.sleep(0.5)  # Wait for publisher to be ready
    
    def move_to_position(self, position):
        """
        Send a position command to the motors
        
        Args:
            position (float): Target position value
        """
        msg = Float64()
        msg.data = position
        self.command_pub.publish(msg)
        rospy.loginfo(f"Sent position command: {position}")
    
    def move_sequence(self, positions, delay=1.0):
        """
        Move through a sequence of positions
        
        Args:
            positions (list): List of position values
            delay (float): Delay between positions in seconds
        """
        rate = rospy.Rate(1.0 / delay)
        for position in positions:
            self.move_to_position(position)
            rate.sleep()


def main():
    """Example usage of the motor interface"""
    rospy.init_node('motor_interface_example', anonymous=True)
    
    interface = MotorInterface()
    
    # Example: Move through a sequence of positions
    positions = [0, 1024, 2048, 3072, 4096, 2048]
    
    rospy.loginfo("Starting motor movement sequence...")
    interface.move_sequence(positions, delay=2.0)
    rospy.loginfo("Sequence complete")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
