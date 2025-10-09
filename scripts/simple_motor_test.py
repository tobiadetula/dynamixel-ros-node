#!/usr/bin/env python3
"""
Simple Motor Test Script

Tests basic motor functionality by moving through a sequence of positions.
"""

import rospy
from std_msgs.msg import Float64
import time


def test_motor():
    """Test motor with a simple movement sequence"""
    rospy.init_node('simple_motor_test', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('motor_command', Float64, queue_size=10)
    rospy.sleep(1.0)  # Wait for publisher to be ready
    
    # Test positions
    positions = [
        0,      # Home position
        1024,   # Quarter turn
        2048,   # Half turn
        3072,   # Three-quarter turn
        4095,   # Full turn
        2048,   # Back to center
        0       # Back to home
    ]
    
    rospy.loginfo("Starting motor test sequence...")
    
    rate = rospy.Rate(0.5)  # 0.5 Hz = 2 seconds per position
    
    for i, pos in enumerate(positions):
        if rospy.is_shutdown():
            break
            
        rospy.loginfo(f"Moving to position {pos} (step {i+1}/{len(positions)})")
        
        msg = Float64()
        msg.data = pos
        pub.publish(msg)
        
        rate.sleep()
    
    rospy.loginfo("Motor test sequence complete!")


if __name__ == '__main__':
    try:
        test_motor()
    except rospy.ROSInterruptException:
        pass
