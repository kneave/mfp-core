#!/usr/bin/env python3
from time import sleep

import rospy
from sensor_msgs.msg import BatteryState

import SDL_Pi_INA3221
 
ina3221 = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)

LEFT_HAND    = 1
RIGHT_HAND   = 2
SERVO_6V_BUS = 3

pub = rospy.Publisher('servo_power', BatteryState, queue_size=10)
rospy.init_node('servo_power_node', anonymous=True)

if __name__ == '__main__':
    try:
        # Loop until disconnected
        while True:
            # Create message
            msg = BatteryState()
            msg.voltage = ina3221.getBusVoltage_V(LEFT_HAND)
            msg.current = ina3221.getCurrent_mA(LEFT_HAND)
            msg.location = "LEFT_HAND"
            rospy.loginfo(msg)
            pub.publish(msg)

            msg = BatteryState()
            msg.voltage = ina3221.getBusVoltage_V(RIGHT_HAND)
            msg.current = ina3221.getCurrent_mA(RIGHT_HAND)
            msg.location = "RIGHT_HAND"
            rospy.loginfo(msg)
            pub.publish(msg)

            msg = BatteryState()
            msg.voltage = ina3221.getBusVoltage_V(SERVO_6V_BUS)
            msg.current = ina3221.getCurrent_mA(SERVO_6V_BUS)
            msg.location = "SERVO_6V_BUS"
            rospy.loginfo(msg)
            pub.publish(msg)

            sleep(0.5)
        
        print("Exiting, controller disconnected.")

    except rospy.ROSInterruptException:
        pass