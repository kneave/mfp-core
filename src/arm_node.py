#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Expander

import math
from time import sleep 
from datetime import datetime
from datetime import timedelta
import sys
import signal

from sensor_msgs.msg import Joy

pub = None # empty global var for the publisher

def handler(signum, frame):
        print("ctrl-c pressed, exiting")
        DisableServos()
        sys.exit()

expander = redboard.PCA9685(address=0x42)
expander.frequency = 50

servoDict = {
    "left_flappy": 0,
    "left_shoulder_tilt": 1,
    "left_shoulder_rotate": 2,
    "left_elbow": 3,
    "left_wrist_left": 4,
    "left_wrist_right": 5,
    "left_hand": 7,
    "right_flappy": 8,
    "right_shoulder_tilt": 9,
    "right_shoulder_rotate": 10,
    "right_elbow": 11,
    "right_wrist_left": 12,
    "right_wrist_right": 13,
    "right_hand": 15
}

axesDict = {
    "lx": 0,
    "ly": 1,
    "lz": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
    "encoder": 6 }

buttonsDict = {
    "S1": 0,
    "S2": 1,
    "S3": 4,
    "ToggleDown": 2,
    "ToggleUp": 3,
    # "Encoder": 5,
    "Trigger": 5,
    "LeftStick": 6,
    "RightStick": 7 }

# Previous value for the trigger, used for debouncing
trigger_prev = False

# Initialise the arm position array
defaultPositions = [ 0.17,  -0.57,  0.14,  0.60,  0.02,  0.00,  0.00,  0.00,
                     0.00,   1.00,  0.00, -1.00,  0.00,  0.00,  0.00,  0.00]

positions = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

minPositions = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  -1.0, -1.0,
                -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  -1.0, -1.0]

maxPositions = [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,
                 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0]

# Index of the position to update, value is the amount to change by.
# Sets the new value if not out of bounds.
def SetPosition(index, value):
    global positions
    newValue = positions[index] + value

    if(newValue > maxPositions[index]):
        newValue = maxPositions[index]
    elif(newValue < minPositions[index]):
        newValue = minPositions[index]

    positions[index] = newValue

# Set the position of each servo
def SetServos():
    global expander
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
    global positions, expander
    # Set the pulse widths for the servos
    expander.servo1_config = 900, 2100
    expander.servo7_config = 700, 2400
    expander.servo9_config = 900, 2100
    expander.servo15_config = 700, 2400

    #  Set the servos to the default positions, do this with delays to prevent overload
    #  Order of initialisation:
    #  elbow (3), 0.4 then to 0.9
    expander.servo3 = positions[3] = defaultPositions[3]
    expander.servo11 = positions[11] = defaultPositions[11]

    sleep(0.25)

    #  elbow_rotate(4) to hand(7) to 0
    expander.servo4 = positions[4] = defaultPositions[4]
    expander.servo5 = positions[5] = defaultPositions[5]
    expander.servo6 = positions[6] = defaultPositions[6]
    expander.servo7 = positions[7] = defaultPositions[7]

    expander.servo12 = positions[12] = defaultPositions[12]
    expander.servo13 = positions[13] = defaultPositions[13]
    expander.servo14 = positions[14] = defaultPositions[14]
    expander.servo15 = positions[15] = defaultPositions[15]

    sleep(0.25)

    #  flappy(0) to 0
    expander.servo0 = positions[0] = defaultPositions[0]
    expander.servo8 = positions[8] = defaultPositions[8]
    sleep(0.25)

    #  upper_rotate(2) to 0.2
    expander.servo2 = positions[2] = defaultPositions[2]
    expander.servo10 = positions[10] = defaultPositions[10]
    sleep(0.25)

    #  shoulder_foreaft(1) to -0.8
    expander.servo1 = positions[1] = defaultPositions[1]
    expander.servo9 = positions[9] = defaultPositions[9]

# Rotation is centred on 0.5 so need to map for that
def MapAxis(input):
    if((input >= 0.43) & (input <= 0.57)):
        return 0
    else:
        return input - 0.5

def ReduceAxis(input):
    return input / 25

# Print the values of all positions to 3 decimal points
def PrintPositions():
    global pub, positions
    
    msg = Expander()   
    msg.servo0 = positions[0]
    msg.servo1 = positions[1]
    msg.servo2 = positions[2]
    msg.servo3 = positions[3]
    msg.servo4 = positions[4]
    msg.servo5 = positions[5]
    msg.servo6 = positions[6]
    msg.servo7 = positions[7]
    msg.servo8 = positions[8]
    msg.servo9 = positions[9]
    msg.servo10 = positions[10]
    msg.servo11 = positions[11]
    msg.servo12 = positions[12]
    msg.servo13 = positions[13]
    msg.servo14 = positions[14]
    msg.servo15 = positions[15]
    
    pub.publish(msg)
    rospy.loginfo(msg)

def callback(data):
    global trigger_prev

    # buttons[3] is the toggle button up on the controller
    if(data.buttons[buttonsDict["ToggleUp"]] == 1):
        # Arm control engaged
        global positions, leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        # check if the trigger has been pulled
        trigger_curr = data.buttons[buttonsDict["Trigger"]]
        trigger_pulled = False
        if trigger_curr != trigger_prev:
            trigger_prev = trigger_curr
            trigger_pulled = trigger_curr

        if(trigger_pulled == 1):
            PrintPositions()

        leftStickButton = data.buttons[buttonsDict["LeftStick"]]
        leftArmControlMode = data.buttons[buttonsDict["S1"]]

        rightStickButton = data.buttons[buttonsDict["RightStick"]]
        rightArmControlMode = data.buttons[buttonsDict["S2"]]

        # Get the data from the analogue sticks
        lx = data.axes[axesDict["lx"]]
        ly = data.axes[axesDict["ly"]]
        lz = data.axes[axesDict["lz"]]

        rx = data.axes[axesDict["rx"]]
        ry = data.axes[axesDict["ry"]]
        rz = data.axes[axesDict["rz"]]

        # We're in left arm control mode, check if arm/hand control needed
        if(leftArmControlMode == 0):
            # control the arm, not the wrist
            SetPosition(servoDict["left_shoulder_rotate"], ReduceAxis(lx))
            
            if(leftStickButton == 0):
                SetPosition(servoDict["left_shoulder_tilt"], -ReduceAxis(ly)) 
            else:
                SetPosition(servoDict["left_elbow"], -ReduceAxis(ly)) 
            
            SetPosition(servoDict["left_flappy"], -ReduceAxis(lz))        
        
        if(leftArmControlMode == 1):
            # controlling the wrist/hand
            handmovement = ReduceAxis(ly)       
            handrotate =   ReduceAxis(lx) * 0.5

            SetPosition(servoDict["left_wrist_left"],  -handmovement + handrotate) 
            SetPosition(servoDict["left_wrist_right"],  handmovement + handrotate)
            SetPosition(servoDict["left_hand"],  ReduceAxis(lz))

        # We're in right arm control mode, check if arm/hand control needed
        if(rightArmControlMode == 0):
            # control the arm, not the wrist
            SetPosition(servoDict["right_shoulder_rotate"], ReduceAxis(rx))
            if(rightStickButton == 0):
                SetPosition(servoDict["right_shoulder_tilt"], ReduceAxis(ry)) 
            else:
                SetPosition(servoDict["right_elbow"], ReduceAxis(ry)) 
                SetPosition(servoDict["right_flappy"], -ReduceAxis(rz))        
        
        if(rightArmControlMode == 1):
            # controlling the wrist/hand
            SetPosition(servoDict["right_wrist_left"], -ReduceAxis(ry)) 
            SetPosition(servoDict["right_wrist_right"],  ReduceAxis(ry))            
            SetPosition(servoDict["right_hand"],  ReduceAxis(rz))

        SetServos()

def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('arm_driver', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    pub = rospy.Publisher('expander_servos_states', Expander, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Arm node listening...")
    signal.signal(signal.SIGINT, handler)
    initialise()    
    listener()
    PrintPositions()
    
