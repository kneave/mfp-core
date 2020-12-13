#!/usr/bin/env python3

import rospy
import redboard
import math

from sensor_msgs.msg import Joy

rb = redboard.RedBoard()

motor1 = 0
motor2 = 0

rb.m0_invert = True
rb.m1_invert = True

# from https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
def steering(x, y):
    # convert to polar
    r = math.hypot(x, y)
    t = math.atan2(y, x)

    # rotate by 45 degrees
    t -= math.pi / 4

    # back to cartesian
    left = r * math.sin(t)
    right = r * math.cos(t)

    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))

    return left, right
    return left, right

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)    
    print(data.axes[1], data.axes[0])

    left, right = steering(data.axes[0], data.axes[1])

    setmotors(left, right)
    
def setmotors(m1, m2):
    rb.m0 = m1
    rb.m1 = m2

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Motor node listening...")
    listener()