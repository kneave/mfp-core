#!/usr/bin/env python3

import rospy
import redboard
import math
from time import sleep 
import sys

from sensor_msgs.msg import Joy

expander = redboard.PCA9685(address=0x42)

# Initialise the arm position array
defaultPositions = [0, -0.8, -0.1,  0.9, 0, 0, 0, 0,
                    0,  0.8,  0.1, -0.9, 0, 0, 0, 0]

positions = [0,0,0,0,0,0,0,0,
             0,0,0,0,0,0,0,0]

minPositions = [-1,-1,-1,-1,-1,-1,-1,-1,
                -1,-1,-1,-1,-1,-1,-1,-1]

maxPositions = [ 1, 1, 1, 1, 1, 1, 1, 1,
                 1, 1, 1, 1, 1, 1, 1, 1]

def SetServos():
    expander.servo0 = positions[0]
    expander.servo1 = positions[1]
    expander.servo2 = positions[2]
    expander.servo3 = positions[3]
    expander.servo4 = positions[4]
    expander.servo5 = positions[5]
    expander.servo6 = positions[6]
    expander.servo7 = positions[7]
    expander.servo8 = positions[8]
    expander.servo9 = positions[9]
    expander.servo10 = positions[10]
    expander.servo11 = positions[11]
    expander.servo12 = positions[12]
    expander.servo13 = positions[13]
    expander.servo14 = positions[14]
    expander.servo15 = positions[15]

#  Set all positions to None then call SetServo
def DisableServos():
    global positions
    for i in range(16):
        positions[i] = None

    SetServos()

def initialise():
    global positions
    #  Set the servos to the default positions, do this with delays to prevent overload
    #  Order of initialisation:
    #  elbow (3), 0.4 then to 0.9
    expander.servo3 = positions[3] = defaultPositions[3]
    expander.servo11 = positions[11] = defaultPositions[11]

    sleep(0.1)

    #  elbow_rotate(4) to hand(7) to 0
    expander.servo4 = positions[4] = defaultPositions[4]
    expander.servo5 = positions[5] = defaultPositions[5]
    expander.servo6 = positions[6] = defaultPositions[6]
    expander.servo7 = positions[7] = defaultPositions[7]

    expander.servo12 = positions[12] = defaultPositions[12]
    expander.servo13 = positions[13] = defaultPositions[13]
    expander.servo14 = positions[14] = defaultPositions[14]
    expander.servo15 = positions[15] = defaultPositions[15]

    sleep(0.1)

    #  flappy(0) to 0
    expander.servo0 = positions[0] = defaultPositions[0]
    expander.servo8 = positions[8] = defaultPositions[8]
    sleep(0.1)

    #  upper_rotate(2) to 0.2
    expander.servo2 = positions[2] = defaultPositions[2]
    expander.servo10 = positions[10] = defaultPositions[10]
    sleep(0.1)

    #  shoulder_foreaft(1) to -0.8
    expander.servo1 = positions[1] = defaultPositions[1]
    expander.servo9 = positions[9] = defaultPositions[9]

# Rotation is centred on 0.5 so need to map for that
def MapAxis(input):
    if((input >= 0.45) & (input <= 0.55)):
        return 0
    else:
        return input - 0.5

def callback(data):
    if(data.buttons[5] == 1):
        DisableServos()
        sys.exit(0)

    if(data.buttons[3] == 1):
        global positions
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        #  if left button pressed, control the elbow instead
        if(data.buttons[7] == 0):
            offset = 0
        else:
            offset = 2
        
        leftRotate = MapAxis(data.axes[4])
        if(leftRotate != 0):
            positions[8 + offset] += (leftRotate / 20)

        positions[9 + offset] += (data.axes[1] / 20)
        positions[10 + offset] += (data.axes[0] / 20)

        #  if right button pressed, control the elbow instead
        if(data.buttons[8] == 0):
            offset = 0
        else:
            offset = 2
        
        rightRotate = MapAxis(data.axes[5])
        if(rightRotate != 0):
            positions[0 + offset] -= (rightRotate / 20)
            # print(rightRotate)

        positions[1 + offset] -= (data.axes[3] / 20)
        positions[2 + offset] -= (data.axes[2] / 20)
        SetServos()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('arm_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Arm node listening...")
    initialise()
    listener()
