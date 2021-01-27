#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
from common.db_schema import BatteryStatusLog
import datetime as dt

SQLITE_DB_PATH = os.getenv('SQLITE_DB_PATH')

class ROSSubscriberBase():

    def __init__(self, name, topic, msg_type):
        self.name = name
        self.topic = topic
        self.msg_type = msg_type

    def run(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.Subscriber(self.topic, self.msg_type, self.callback)
        rospy.spin()

    def callback(self, data):
        raise Exception('Not Implemented.')


class BatteryStateSubscriber(ROSSubscriberBase):

    def __init__(self):
        self.local_db = BatteryStatusLog(SQLITE_DB_PATH)
        super().__init__('battery_state_subscriber', 'fake_battery_state', BatteryState)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.local_db.insert({
            'timestamp': dt.datetime.now(),
            'current': data.current,
            'voltage': data.voltage,
            'power_supply_status': data.power_supply_status,
        })

if __name__ == '__main__':
    BatteryStateSubscriber().run()
