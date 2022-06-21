#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import BatteryState

def callback(msg):
    if msg.location == "LEFT_HAND":
        print(f'{msg.location}, {msg.voltage:.2f}, {msg.current:.2f}')

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('servo_power_monitor', anonymous=True)
    rospy.Subscriber('servo_power', BatteryState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Servo monitor node listening...")
    listener()
