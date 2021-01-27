#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import sqlite3
import datetime as dt


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    
    conn = sqlite3.connect('example.db')
    c = conn.cursor()
    c.execute(f"INSERT INTO battery_status_log VALUES ('{dt.datetime.now()}', '{data.current}', '{data.voltage}', '{data.power_supply_status}')")
    conn.commit()
    conn.close()

def listener():

    conn = sqlite3.connect('example.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS battery_status_log (date timestamp, current real, voltage real, power_supply_mode int)''')
    conn.commit()
    conn.close()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('battery_state_subscriber', anonymous=True)

    rospy.Subscriber("fake_battery_state", BatteryState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
