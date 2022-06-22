#!/usr/bin/env python3

import rospy
import redboard
from rosredboard.msg import Expander
import signal
import sys
import time
from sensor_msgs.msg import BatteryState

#  Get the expander address
expander_address = rospy.get_param('/expander_address', 66)
rospy.loginfo("Expander address: " + str(expander_address))

expander = redboard.PCA9685(address=expander_address)
expander.frequency = 50

expander.servo7_config = 700, 2400
expander.servo15_config = 700, 2400

left_hand_powers  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
left_hand_powers_index = 0
right_hand_powers = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
right_hand_powers_index = 0

# stores the current positions of all the servos
positions = Expander()
positions.servo7  = -0.42
positions.servo15  = 0.32

def Lerp(start, end, ratio):
    if start == end:
        return start

    total_diff = abs(start) + abs(end)

    if start < end:
        return start + (total_diff * ratio)
    else:
        return start - (total_diff * ratio)

def AveragePowers(array):
    sumcurrent = 0
    for i in array:
        sumcurrent += i
    return sumcurrent / 5

def InitServos():
    global positions
    expander.servo7 = positions.servo7
    expander.servo15 = positions.servo15

def DisableServos():
    global expander
    expander.servo7 = None
    expander.servo15 = None

def handler(signum, frame):
    print("ctrl-c pressed, exiting")
    DisableServos()
    sys.exit()

# target positions, time in seconds
def LerpPositions(msg, time_to_move):
    global positions, expander
    global left_hand_powers, left_hand_powers_index, right_hand_powers, right_hand_powers_index

    # create a loop
    #   each iteration, move each position a bit closer
    # go with 50hz as a rate for finer control
    rate = 50
    delayPerLoop = time_to_move / rate
    iterations_needed = time_to_move * rate

    left_limited = False
    right_limited = False

    # how many iterations do we need?
    for x in range(iterations_needed + 1):
        if (left_limited == True)& (right_limited == True):
            print("Exiting for loop, limited")
            break

        ratio = x / iterations_needed

        if (msg.servo7 != 999.0) & (left_limited == False):            
            averages = AveragePowers(left_hand_powers)
            print(averages)
            
            # check average current usage and stop if high
            if (msg.servo7 > positions.servo7):
                # hand is closing, if average low continue. Else setpositions to current
                
                left_hand_limit = 300
                if (averages < left_hand_limit):
                    expander.servo7 = Lerp(positions.servo7, msg.servo7, ratio)
                else:
                    print("current limiting left, backing off")
                    while left_limited == False:
                        averages = AveragePowers(left_hand_powers)
                        if averages > left_hand_limit:
                            expander.servo7 = expander.servo7 - 0.02
                            print(expander.servo7)
                            time.sleep(0.05)
                        else:
                            print("Backed off left.")
                            left_limited = True
                            break
                        
            else:
                # hand opening
                expander.servo7 = Lerp(positions.servo7, msg.servo7, ratio)

        if (msg.servo15 != 999.0) & (right_limited == False):            
            averages = AveragePowers(right_hand_powers)
            print(f'r avg:{averages:.4f}')
            
            # check average current usage and stop if high
            if (msg.servo15 > positions.servo15):
                # hand is closing, if average low continue. Else setpositions to current
                
                right_hand_limit = 300
                if (averages < right_hand_limit):
                    expander.servo15 = Lerp(positions.servo15, msg.servo15, ratio)
                    print(f'r mov:{ratio:.4f}')
                    
                else:
                    print("right current limiting, backing off")
                    while right_limited == False:
                        averages = AveragePowers(right_hand_powers)
                        if averages > right_hand_limit:
                            expander.servo15 = expander.servo15 - 0.02
                            print(f'right hand: {expander.servo15:.2f}')
                            time.sleep(0.05)
                        else:
                            print("Backed off right.")
                            right_limited = True
                            break
            else:
                # hand opening
                expander.servo15 = Lerp(positions.servo15, msg.servo15, ratio)

        time.sleep(delayPerLoop)

    # set positions array to current values
    positions.servo7 = expander.servo7
    positions.servo15 = expander.servo15

def expander_callback(msg):
    # rospy.loginfo(msg)
    LerpPositions(msg, 1)

def IncIndex(input):
    if input == 9:
        return 0
    else:
        return input + 1

def battery_callback(msg):
    global left_hand_powers, left_hand_powers_index, right_hand_powers, right_hand_powers_index

    if(msg.location=="LEFT_HAND"):
        left_hand_powers[left_hand_powers_index] = msg.current
        left_hand_powers_index = IncIndex(left_hand_powers_index)
    if(msg.location=="RIGHT_HAND"):
        right_hand_powers[right_hand_powers_index] = msg.current
        right_hand_powers_index = IncIndex(right_hand_powers_index)

def listener():
    rospy.init_node('hand_control_node', anonymous=True)
    rospy.Subscriber('/redboard/expander', Expander, expander_callback)
    rospy.Subscriber('servo_power', BatteryState, battery_callback)

    rospy.spin()

if __name__ == '__main__':
    print("Hand control node listening...")
    signal.signal(signal.SIGINT, handler)
    InitServos()
    listener()
