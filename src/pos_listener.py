#!/usr/bin/env python3

import rospy
import math
from rosredboard.msg import Expander

def callback(msg):
    print(f'{msg.servo0:.2f}, {msg.servo1:.2f}, {msg.servo2:.2f}, {msg.servo3:.2f}, {msg.servo4:.2f}, {msg.servo5:.2f}, {msg.servo6:.2f}, {msg.servo7:.2f}, {msg.servo8:.2f}, {msg.servo9:.2f}, {msg.servo10:.2f}, {msg.servo11:.2f}, {msg.servo12:.2f}, {msg.servo13:.2f}, {msg.servo14:.2f}, {msg.servo15:.2f}')

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('arm_position_monitor', anonymous=True)
    rospy.Subscriber('expander_servos_states', Expander, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Arm position monitor node listening...")
    listener()