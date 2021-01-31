#!/usr/bin/env python3

import rospy
from rospy import Publisher, Rate
from sensor_msgs.msg import BatteryState
from common.fake_battery import FakeBattery

# For type hinting
from typing import Any


class ROSPublisherBase:
    ''' Generic base class which can be inherited for creating ROS Publishers '''

    def __init__(self, name: str, topic: str, msg_type: Any, rate: float) -> None:
        self.name: str = name
        self.topic: str = topic
        self.msg_type: Any = msg_type
        self.rate_float: float = rate

    def run(self) -> None:
        try:
            # Initialize ROS node and a Publisher to a specific topic and message type
            self.pub: Publisher = Publisher(self.topic, self.msg_type, queue_size=10)
            rospy.init_node(self.name, anonymous=True)
            self.rate: Rate = Rate(self.rate_float) # hz
            self.publish_loop()
        except rospy.ROSInterruptException:
            pass

    def publish_loop(self) -> None:
        ''' Must be implemented by child classes '''
        raise Exception('Not Implemented.')


class BatteryStatePublisher(ROSPublisherBase):
    ''' 
        Battery State Publisher which takes input from a Fake battery and publishes
        it's state (current, voltage, power_supply_status) to 'fake_battery_state' topic.
    '''

    def __init__(self) -> None:
        self.fake_battery: FakeBattery = FakeBattery(capacity=5, rate=0.5)
        super().__init__('battery_state_publisher', 'fake_battery_state', BatteryState, 0.5)

    def publish_loop(self) -> None:
        ''' 
            Main loop which will publish to 'fake_battery_state' topic at frequency 0.5 Hz
            Every iteration of the loop updates the state of the fake battery and then publishes
            it to the specified topic.
        '''
        while not rospy.is_shutdown():
            self.fake_battery.update_state()

            bs_value: BatteryState = BatteryState()
            bs_value.voltage = self.fake_battery.voltage
            bs_value.current = self.fake_battery.current
            bs_value.power_supply_status = self.fake_battery.power_supply_status

            # Log published values for debugging
            rospy.loginfo(bs_value)

            self.pub.publish(bs_value)
            self.rate.sleep()


if __name__ == '__main__':
    BatteryStatePublisher().run()
