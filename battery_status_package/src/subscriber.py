#!/usr/bin/env python3

import os
import datetime as dt
import rospy
from rospy import Subscribers
from sensor_msgs.msg import BatteryState
from common.db_schema import BatteryStatusLog

# For type hinting
from typing import Any

# Initialize Environment variables
SQLITE_DB_PATH: str = os.getenv('SQLITE_DB_PATH')


class ROSSubscriberBase:
    ''' Generic base class which can be inherited for creating ROS Subscribers '''

    def __init__(self, name: str, topic: str, msg_type: Any) -> None:
        self.name: str = name
        self.topic: str = topic
        self.msg_type: Any = msg_type

    def run(self) -> None:
        rospy.init_node(self.name, anonymous=True)
        Subscriber(self.topic, self.msg_type, self.callback)
        rospy.spin()

    def callback(self, data: Any) -> None:
        ''' Must be implemented by child classes '''
        raise Exception('Not Implemented.')


class BatteryStateSubscriber(ROSSubscriberBase):
    ''' Generic base class which can be inherited for creating ROS Subscribers '''

    def __init__(self) -> None:
        self.local_db: BatteryStatusLog = BatteryStatusLog(SQLITE_DB_PATH)
        super().__init__('battery_state_subscriber', 'fake_battery_state', BatteryState)

    def callback(self, data: Any) -> None:
        ''' 
            Callback is called everytime there is a new message for the topic 'fake_battery_state'
        '''

        # Log received data for debugging
        rospy.loginfo(data)

        # Insert Battery State information to a local database
        self.local_db.insert({
            'timestamp': dt.datetime.now(),
            'current': data.current,
            'voltage': data.voltage,
            'power_supply_status': data.power_supply_status,
        })

if __name__ == '__main__':
    BatteryStateSubscriber().run()
