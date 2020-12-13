#!/usr/bin/env python3

import rospy
import redboard
import math

from sensor_msgs.msg import Joy

rb = redboard.RedBoard()

pan = pan_centre = -0.2
tilt = tilt_centre = 0

pan_min = -0.5 + pan_centre
pan_max = 0.5 + pan_centre

tilt_min = -0.5 + tilt_centre
tilt_max = 0.5 + tilt_centre

def reset():
    global pan, tilt
    pan = pan_centre
    tilt = tilt_centre
    rb.s20 = pan
    rb.s21 = tilt

def callback(data):
    if(data.buttons[2] == 1):
        print("Resetting head position")
        reset()
    else:
        print(data.axes[3], data.axes[4])
        setservos(data.axes[3] / 20, data.axes[4] / 20)
        
def setservos(pan_diff, tilt_diff):
    global pan, tilt
    pan += pan_diff
    tilt += tilt_diff

    if(pan > pan_max):
        pan = pan_max
    elif(pan < pan_min):
        pan = pan_min

    if(tilt > tilt_max):
        tilt = tilt_max
    elif(tilt < tilt_min):
        tilt = tilt_min

    rb.s20 = pan
    rb.s21 = tilt

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('head_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Head node listening...")
    reset()
    listener()