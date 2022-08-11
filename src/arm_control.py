#!/usr/bin/env python3

import rospy
import redboard

from sensor_msgs.msg import Joy
from rosredboard.msg import Expander

import signal
import sys
import time

#  Get the expander address
expander_address = rospy.get_param('/expander_address', 66)
rospy.loginfo("Expander address: " + str(expander_address))
pospub = None

rbex = redboard.PCA9685(address=expander_address)
rbex.frequency = 50

# Set the pulse widths for the servos
rbex.servo1_config = 900, 2100
rbex.servo4_config = 700, 2400
rbex.servo5_config = 700, 2400
rbex.servo9_config = 900, 2100
rbex.servo12_config = 700, 2400
rbex.servo13_config = 700, 2400

# Previous value for the trigger, used for debouncing
trigger_prev = False
lerpingInProgress = False

axesDict = {
    "lx": 0,
    "ly": 1,
    "lz": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
    "encoder": 6 }

xboxAxesDict = {
    "lx": 0,
    "ly": 1,
    "lt": 2, # -1:1, zero at start then 1 at rest
    "rx": 3,
    "ry": 4,
    "rt": 5, # -1:1, zero at start then 1 at rest
    "d-lr": 6,
    "d-ud": 7 }

xboxButtonsDict = {
    "A": 0,
    "B": 1,
    "X": 2,
    "Y": 3,
    "LB": 4,
    "RB": 5,
    "View": 6,
    "Menu": 7,
    "Meatball": 8,
    "LS": 9,
    "RS": 10 }

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


# stores the current positions of all the servos
positions = Expander()

# 0.09, 1.00, 0.25, 0.89, -0.26, 0.36, 0.00

positions.servo0  =  0.09
positions.servo1  =  -0.48
positions.servo2  =  0.12
positions.servo3  =  0.89
positions.servo4  =  0.22
positions.servo5  = -0.22

positions.servo8  =  0.09
positions.servo9  =  0.95
positions.servo10 =  0.00
positions.servo11 = -0.93
positions.servo12 =  0.50
positions.servo13 = -0.19 

# store the default states
rest_positions = positions 

def Lerp(start, end, ratio):
    if start == end:
        return start

    return (start + (end - start) * ratio)

def InitServos():
    global positions, rbex
    rbex.servo0 = positions.servo0
    rbex.servo8 = positions.servo8
    time.sleep(0.1)
    rbex.servo1 = positions.servo1
    rbex.servo9 = positions.servo9
    time.sleep(0.1)
    rbex.servo2 = positions.servo2
    rbex.servo10 = positions.servo10
    time.sleep(0.1)
    rbex.servo3 = positions.servo3
    rbex.servo11 = positions.servo11
    time.sleep(0.1)
    rbex.servo4 = positions.servo4
    rbex.servo12 = positions.servo12
    time.sleep(0.1)
    rbex.servo5 = positions.servo5
    rbex.servo13 = positions.servo13


# Set the position of each servo
def SetServos():
    global rbex, positions

    # left arm
    rbex.servo0 = positions.servo0
    rbex.servo1 = positions.servo1
    rbex.servo2 = positions.servo2
    rbex.servo3 = positions.servo3
    rbex.servo4 = positions.servo4
    rbex.servo5 = positions.servo5
    
    # right arm
    rbex.servo8 = positions.servo8
    rbex.servo9 = positions.servo9
    rbex.servo10 = positions.servo10
    rbex.servo11 = positions.servo11
    rbex.servo12 = positions.servo12
    rbex.servo13 = positions.servo13

def DisableServos():
    global expander
    rbex.servo0 = None
    time.sleep(0.1)
    rbex.servo1 = None
    time.sleep(0.1)
    rbex.servo2 = None
    time.sleep(0.1)
    rbex.servo3 = None
    time.sleep(0.1)
    rbex.servo4 = None
    time.sleep(0.1)
    rbex.servo5 = None
    time.sleep(0.1)
    rbex.servo6 = None
    time.sleep(0.1)
    rbex.servo8 = None
    time.sleep(0.1)
    rbex.servo9 = None
    time.sleep(0.1)
    rbex.servo10 = None
    time.sleep(0.1)
    rbex.servo11 = None
    time.sleep(0.1)
    rbex.servo12 = None
    time.sleep(0.1)
    rbex.servo13 = None
    time.sleep(0.1)
    rbex.servo14 = None

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()

def joy_callback(data):
    global trigger_prev, positions, lerpingInProgress
    if lerpingInProgress == True:
        # print("lerping in progress")
        return

    isXbox = False       #   Are we using an xbox controller?

    if(data.buttons[buttonsDict["ToggleUp"]] == 1) or (isXbox == True):
        # Arm control engaged
        global positions, leftLastClicked, leftPrev, rightLastClicked, rightPrev
        # print(data.axes[0], data.axes[1], data.axes[2], data.axes[3])

        if isXbox == True:
            lx = -data.axes[xboxAxesDict["lx"]]
            ly = -data.axes[xboxAxesDict["ly"]]

            rx =  data.axes[xboxAxesDict["rx"]]
            ry =  data.axes[xboxAxesDict["ry"]]

            reduction = 50

            leftAxisX = ReduceAxis(lx, reduction)
            leftAxisY = ReduceAxis(ly, reduction)
        
            rightAxisX = ReduceAxis(rx, reduction)
            rightAxisY = ReduceAxis(ry, reduction)
        
            trigger_curr = data.buttons[xboxButtonsDict["Meatball"]]

            leftStickButton = data.buttons[xboxButtonsDict["LS"]]
            leftControlArm =  data.buttons[xboxButtonsDict["LB"]]
            leftControlHand = TriggerButton(data.axes[xboxAxesDict["lt"]])

            rightStickButton = data.buttons[xboxButtonsDict["RS"]]
            rightControlArm =  data.buttons[xboxButtonsDict["RB"]]
            rightControlHand = TriggerButton(data.axes[xboxAxesDict["rt"]])

        else:
            lx = data.axes[axesDict["lx"]]
            ly = data.axes[axesDict["ly"]]
            lz = data.axes[axesDict["lz"]]

            rx = data.axes[axesDict["rx"]]
            ry = data.axes[axesDict["ry"]]
            rz = data.axes[axesDict["rz"]]

            reduction = 25

            leftAxisX = ReduceAxis(lx, reduction)
            leftAxisY = ReduceAxis(ly, reduction)
            leftAxisZ = ReduceAxis(lz, reduction)

            rightAxisX = ReduceAxis(rx, reduction)
            rightAxisY = ReduceAxis(ry, reduction)
            rightAxisZ = ReduceAxis(rz, reduction)

            trigger_curr = data.buttons[buttonsDict["Trigger"]]

            leftStickButton = data.buttons[buttonsDict["LeftStick"]]
            leftControlArm = data.buttons[buttonsDict["ToggleUp"]]
            leftControlHand = data.buttons[buttonsDict["S1"]]

            rightStickButton = data.buttons[buttonsDict["RightStick"]]
            rightControlArm = data.buttons[buttonsDict["ToggleUp"]]
            rightControlHand = data.buttons[buttonsDict["S2"]]

        # check if the trigger has been pulled
        trigger_pulled = False
        if trigger_curr != trigger_prev:
            trigger_prev = trigger_curr
            trigger_pulled = trigger_curr

        if(trigger_pulled == 1):
            PrintPositions()

        # We're in left arm control mode, check if arm/hand control needed
        if(leftControlArm == 1):
            if(leftControlHand == 0):
                # control the arm, not the wrist
                                
                if isXbox == True:
                    print(leftAxisY)
                    positions.servo2 += leftAxisX
                    positions.servo1 += leftAxisY
                    positions.servo0 += rightAxisX
                    positions.servo3 += rightAxisY   
                else:
                    if(leftStickButton == 0):
                        positions.servo1 -= leftAxisY 
                    else:
                        positions.servo3 -= leftAxisY
                    positions.servo0 -= leftAxisZ      
                    positions.servo2 += leftAxisX      
            
            if(leftControlHand == 1):
                # controlling the wrist/hand      
                handrotate = leftAxisX * 0.5
                positions.servo4 += (-leftAxisY + handrotate) 
                positions.servo5 += (leftAxisY + handrotate)

        # We're in right arm control mode, check if arm/hand control needed
        if(rightControlArm == 1):
            if(rightControlHand == 0):                           
                if isXbox == True:
                    positions.servo10 += leftAxisX
                    positions.servo9  += leftAxisY
                    positions.servo8  += rightAxisX
                    positions.servo11 += rightAxisY        
                else:
                    if(rightStickButton == 0):
                        positions.servo9 += rightAxisY 
                    else:
                        positions.servo11 += rightAxisY 
                    positions.servo8 -= rightAxisZ    
                    positions.servo10 += rightAxisX

            if(rightControlHand == 1):
                # controlling the wrist/hand
                
                if(isXbox == True):
                    handrotate = leftAxisX * 0.5
                    positions.servo12 += (-leftAxisY + handrotate) 
                    positions.servo13 += (leftAxisY + handrotate)            
                else:
                    handrotate = rightAxisX * 0.5
                    positions.servo12 += (-rightAxisY + handrotate) 
                    positions.servo13 += (rightAxisY + handrotate)            

        SetServos()

def ReduceAxis(input, amount):
    return input / amount

def TriggerButton(input):
    if input < 0.75:
        return 1
    else:
        return 0

# Rotation is centred on 0.5 so need to map for that
def MapAxis(input):
    if((input >= 0.43) & (input <= 0.57)):
        return 0
    else:
        return input - 0.5

# Print the values of all positions to 3 decimal points
def PrintPositions():
    global pospub, rbex
    
    msg = Expander()   
    msg.servo0 = rbex.servo0
    msg.servo1 = rbex.servo1
    msg.servo2 = rbex.servo2
    msg.servo3 = rbex.servo3
    msg.servo4 = rbex.servo4
    msg.servo5 = rbex.servo5
    msg.servo8 = rbex.servo8
    msg.servo9 = rbex.servo9
    msg.servo10 = rbex.servo10
    msg.servo11 = rbex.servo11
    msg.servo12 = rbex.servo12
    msg.servo13 = rbex.servo13
    
    pospub.publish(msg)
    # rospy.loginfo(msg)

# target positions, time in seconds
def LerpPositions(msg, time_to_move):
    global positions, rbex, lerpingInProgress

    # print("Lerping from -> to")
    # print(f'{positions.servo0} -> {msg.servo0}')
    # print(f'{positions.servo1} -> {msg.servo1}')
    # print(f'{positions.servo2} -> {msg.servo2}')
    # print(f'{positions.servo3} -> {msg.servo3}')
    # print(f'{positions.servo4} -> {msg.servo4}')
    # print(f'{positions.servo5} -> {msg.servo5}')
    # print(f'{positions.servo7} -> {msg.servo7}')
    
    # create a loop
    #   each iteration, move each position a bit closer
    # go with 25hz as a rate
    rate = 25
    delayPerLoop = time_to_move / rate
    iterations_needed = time_to_move * rate

    lerpingInProgress = True
    
    # how many iterations do we need?
    for x in range(iterations_needed + 1):
        ratio = x / iterations_needed

        if not msg.servo0 > 1.0:
            rbex.servo0 = Lerp(positions.servo0, msg.servo0, ratio)
        if not msg.servo1 > 1.0:
            rbex.servo1 = Lerp(positions.servo1, msg.servo1, ratio)
        if not msg.servo2 > 1.0:
            rbex.servo2 = Lerp(positions.servo2, msg.servo2, ratio)
        if not msg.servo3 > 1.0:
            rbex.servo3 = Lerp(positions.servo3, msg.servo3, ratio)
        if not msg.servo4 > 1.0:
            rbex.servo4 = Lerp(positions.servo4, msg.servo4, ratio)
        if not msg.servo5 > 1.0:
            rbex.servo5 = Lerp(positions.servo5, msg.servo5, ratio)
        if not msg.servo8 > 1.0:
            rbex.servo8 = Lerp(positions.servo8, msg.servo8, ratio)
        if not msg.servo9 > 1.0:
            rbex.servo9 = Lerp(positions.servo9, msg.servo9, ratio)
        if not msg.servo10 > 1.0:
            rbex.servo10 = Lerp(positions.servo10, msg.servo10, ratio)
        if not msg.servo11 > 1.0:
            rbex.servo11 = Lerp(positions.servo11, msg.servo11, ratio)
        if not msg.servo12 > 1.0:
            rbex.servo12 = Lerp(positions.servo12, msg.servo12, ratio)
        if not msg.servo13 > 1.0:
            rbex.servo13 = Lerp(positions.servo13, msg.servo13, ratio)

        time.sleep(delayPerLoop)

    # set positions array to current values
    positions.servo0 = rbex.servo0
    positions.servo1 = rbex.servo1
    positions.servo2 = rbex.servo2
    positions.servo3 = rbex.servo3
    positions.servo4 = rbex.servo4
    positions.servo5 = rbex.servo5
    positions.servo8 = rbex.servo8
    positions.servo9 = rbex.servo9
    positions.servo10 = rbex.servo10
    positions.servo11 = rbex.servo11
    positions.servo12 = rbex.servo12
    positions.servo13 = rbex.servo13

    lerpingInProgress = False

def expander_callback(msg):
    # rospy.loginfo(msg)
    LerpPositions(msg, 1)

def listener():
    global pospub
    rospy.init_node('redboard_servo_expander_driver', anonymous=True)
    rospy.Subscriber('/redboard/expander', Expander, expander_callback)
    rospy.Subscriber('joy', Joy, joy_callback)
    pospub = rospy.Publisher('expander_servos_states', Expander, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    print("Arm controller node online.")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
