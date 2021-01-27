#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
from .fake_battery import FakeBattery

class ROSPublisherBase():

    def __init__(self, name, topic, msg_type, rate):
        self.name = name
        self.topic = topic
        self.msg_type = msg_type
        self.rate = rospy.Rate(rate) # hz

    def run(self):
        try:
            self.pub = rospy.Publisher(self.topic, self.msg_type, queue_size=10)
            rospy.init_node(self.name, anonymous=True)
            self.publish_loop()
        except rospy.ROSInterruptException:
            pass

    def publish_loop(self):
        raise Exception('Not Implemented.')


class BatteryStatePublisher(ROSPublisherBase):

    def __init__(self):
        self.fake_battery = FakeBattery(capacity=1, rate=0.5)
        super().__init__('battery_state_publisher', 'fake_battery_state', BatteryState, 0.5)

    def publish_loop(self):
        while not rospy.is_shutdown():
            self.fake_battery.update_state()
            bs_value = BatteryState()
            bs_value.voltage = self.fake_battery.voltage
            bs_value.current = self.fake_battery.current
            bs_value.power_supply_status = self.fake_battery.power_supply_status
            rospy.loginfo(bs_value)
            self.pub.publish(bs_value)
            self.rate.sleep()

if __name__ == '__main__':
    BatteryStatePublisher().run()
