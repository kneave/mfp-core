#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Expander
import signal
import sys
import time

#  Get the expander address
expander_address = rospy.get_param('/expander_address', 66)
rospy.loginfo("Expander address: " + str(expander_address))

expander = redboard.PCA9685(address=expander_address)
expander.frequency = 50

expander.servo1_config = 900, 2100
expander.servo9_config = 900, 2100

# stores the current positions of all the servos
positions = Expander()
positions.servo0  = 0.17
positions.servo1  = -0.57
positions.servo2  = 0.14
positions.servo3  = 0.6
positions.servo4  = 0.02
positions.servo5  = 0
positions.servo6  = 0

def Lerp(start, end, ratio):
    if start == end:
        return start

    total_diff = abs(start) + abs(end)

    if start < end:
        return start + (total_diff * ratio)
    else:
        return start - (total_diff * ratio)

def InitServos():
    global positions
    expander.servo0 = positions.servo0
    time.sleep(0.1)
    expander.servo1 = positions.servo1
    time.sleep(0.1)
    expander.servo2 = positions.servo2
    time.sleep(0.1)
    expander.servo3 = positions.servo3
    time.sleep(0.1)
    expander.servo4 = positions.servo4
    time.sleep(0.1)
    expander.servo5 = positions.servo5
    time.sleep(0.1)
    expander.servo6 = positions.servo6

def DisableServos():
    global expander
    expander.servo0 = None
    time.sleep(0.1)
    expander.servo1 = None
    time.sleep(0.1)
    expander.servo2 = None
    time.sleep(0.1)
    expander.servo3 = None
    time.sleep(0.1)
    expander.servo4 = None
    time.sleep(0.1)
    expander.servo5 = None
    time.sleep(0.1)
    expander.servo6 = None
    time.sleep(0.1)
    expander.servo8 = None
    time.sleep(0.1)
    expander.servo9 = None
    time.sleep(0.1)
    expander.servo10 = None
    time.sleep(0.1)
    expander.servo11 = None
    time.sleep(0.1)
    expander.servo12 = None
    time.sleep(0.1)
    expander.servo13 = None
    time.sleep(0.1)
    expander.servo14 = None

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()

# target positions, time in seconds
def LerpPositions(msg, time_to_move):
    global positions, expander

    # create a loop
    #   each iteration, move each position a bit closer
    # go with 25hz as a rate
    rate = 25
    delayPerLoop = time_to_move / rate
    iterations_needed = time_to_move * rate

    # how many iterations do we need?
    for x in range(iterations_needed + 1):
        ratio = x / iterations_needed

        if msg.servo0 != 999.0:
            expander.servo0 = Lerp(positions.servo0, msg.servo0, ratio)
        if msg.servo1 != 999.0:
            expander.servo1 = Lerp(positions.servo1, msg.servo1, ratio)
        if msg.servo2 != 999.0:
            expander.servo2 = Lerp(positions.servo2, msg.servo2, ratio)
        if msg.servo3 != 999.0:
            expander.servo3 = Lerp(positions.servo3, msg.servo3, ratio)
        if msg.servo4 != 999.0:
            expander.servo4 = Lerp(positions.servo4, msg.servo4, ratio)
        if msg.servo5 != 999.0:
            expander.servo5 = Lerp(positions.servo5, msg.servo5, ratio)
        if msg.servo6 != 999.0:
            expander.servo6 = Lerp(positions.servo6, msg.servo6, ratio)
        if msg.servo8 != 999.0:
            expander.servo8 = Lerp(positions.servo8, msg.servo8, ratio)
        if msg.servo9 != 999.0:
            expander.servo9 = Lerp(positions.servo9, msg.servo9, ratio)
        if msg.servo10 != 999.0:
            expander.servo10 = Lerp(positions.servo10, msg.servo10, ratio)
        if msg.servo11 != 999.0:
            expander.servo11 = Lerp(positions.servo11, msg.servo11, ratio)
        if msg.servo12 != 999.0:
            expander.servo12 = Lerp(positions.servo12, msg.servo12, ratio)
        if msg.servo13 != 999.0:
            expander.servo13 = Lerp(positions.servo13, msg.servo13, ratio)
        if msg.servo14 != 999.0:
            expander.servo14 = Lerp(positions.servo14, msg.servo14, ratio)

        time.sleep(delayPerLoop)

    # set positions array to current values
    positions.servo0 = expander.servo0
    positions.servo1 = expander.servo1
    positions.servo2 = expander.servo2
    positions.servo3 = expander.servo3
    positions.servo4 = expander.servo4
    positions.servo5 = expander.servo5
    positions.servo6 = expander.servo6
    positions.servo8 = expander.servo8
    positions.servo9 = expander.servo9
    positions.servo10 = expander.servo10
    positions.servo11 = expander.servo11
    positions.servo12 = expander.servo12
    positions.servo13 = expander.servo13
    positions.servo14 = expander.servo14

def expander_callback(msg):
    # rospy.loginfo(msg)
    LerpPositions(msg, 1)


def listener():
    rospy.init_node('redboard_servo_expander_driver', anonymous=True)
    rospy.Subscriber('/redboard/expander', Expander, expander_callback)

    rospy.spin()

if __name__ == '__main__':
    print("RedBoard servo expander node listening...")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
