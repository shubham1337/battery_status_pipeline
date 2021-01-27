#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import datetime as dt
import math
import random
from random import gauss

def talker():
    pub = rospy.Publisher('fake_battery_state', BatteryState, queue_size=10)
    rospy.init_node('battery_state_publisher', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    curr_voltage = 0
    curr_current = 0
    curr_power_supply_status = 2
    while not rospy.is_shutdown():
        bs_value = BatteryState()
        if curr_power_supply_status == 1:
            curr_voltage = 0.001
            curr_current = 1
        if curr_power_supply_status == 2:
            curr_voltage = 0.01
            curr_current = -abs(gauss(0.25, math.sqrt(0.8)))
            if random.random() > 0.8:
                curr_current = -4
        bs_value.voltage = curr_voltage
        bs_value.current = curr_current
        bs_value.power_supply_status = curr_power_supply_status
        rospy.loginfo(bs_value)
        pub.publish(bs_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
