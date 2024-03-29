#!/usr/bin/env python3

import rospy
import redboard
import math
import time

from sensor_msgs.msg import Joy

# time in seconds in not recieving a message before stopping
msg_timeout = rospy.Duration(0.50)
last_msg_received = rospy.Time.from_sec(time.time())

rb = redboard.RedBoard()

rb.m0_invert = True         # front left 
rb.m1_invert = False        # front right
rb.m2_invert = True         # rear left
rb.m3_invert = False        # rear right

WHEEL_RADIUS = 30
WHEEL_SEPARATION_WIDTH  = 195 
WHEEL_SEPARATION_LENGTH = 150
WHEEL_GEOMETRY = (WHEEL_SEPARATION_LENGTH + WHEEL_SEPARATION_WIDTH) / 2

# from https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
def steering(x, y, rot):
    x = x * -1
    y = y * -1
    rot = rot * -1
    
    front_left = (x - y - rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot) # * WHEEL_GEOMETRY) / WHEEL_RADIUS

    return front_left, front_right, back_left, back_right

def callback(data):
    global last_msg_received
    # rospy.loginfo(rospy.get_caller_id() + 'RCVD: %s', data)
    last_msg_received = rospy.Time.now()

    isXbox = False

    if isXbox == True:
        # If buffer buttons presed, arms being controlled so return
        if (data.buttons[4] == 1) or (data.buttons[5] == 1):
            return

        control_movement = True
        x, y, z = data.axes[1], data.axes[3], data.axes[0]

    else:
        control_movement = data.buttons[2] == 1
        x, y, z = -data.axes[1], data.axes[0], data.axes[2]

    if(control_movement == True):
        # print(data.axes[0], data.axes[1])
        front_left, front_right, back_left, back_right = steering(x, y, z)
        # print(front_left, front_right, back_left, back_right)

        # Buttons are on when down so this makes sense in the physical world
        if(data.buttons[4] == 1):
            # Low speed, halve values
            front_left = front_left * 0.33
            front_right = front_right * 0.33
            back_left = back_left * 0.33
            back_right = back_right * 0.33
        else:
            front_left = front_left * 0.66
            front_right = front_right * 0.66
            back_left = back_left * 0.66
            back_right = back_right * 0.66

        setmotors(front_left, front_right, back_left, back_right)
    else:
        #  print("Motors not enabled.")
        setmotors(0, 0, 0, 0)

def setmotors(m0, m1, m2, m3):
    rb.m0 = m0
    rb.m1 = m1
    rb.m2 = m2
    rb.m3 = m3


def timeout_check(event):
    timediff = rospy.Time.now() - last_msg_received
    if(timediff > msg_timeout):
        print("Timed out: " + str(rospy.Time.now()) + ", " + str(last_msg_received) + ", " + str(timediff))
        setmotors(0, 0, 0, 0)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # Set up the timer-based timeout check
    rospy.Timer(msg_timeout, timeout_check)

    rospy.spin()

if __name__ == '__main__':
    print("Motor node listening...")
    listener()
