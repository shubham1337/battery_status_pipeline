
import unittest

from common.fake_battery import FakeBattery


class TestFakeBattery(unittest.TestCase):
    ''' Test Cases for testing various state changes of the FakeBattery module '''

    def test_first_state(self):
        ''' Test first state of battery after initialization '''

        fb: FakeBattery = FakeBattery(capacity=1.0, rate=1)
        fb.update_state()
        self.assertEqual(fb.voltage, 0)
        self.assertTrue(fb.current < 0)
        self.assertEqual(fb.power_supply_status, 3)

    def test_second_state(self):
        ''' Test second state of battery after initialization, it must have started charging '''

        fb: FakeBattery = FakeBattery(capacity=1.0, rate=1)
        fb.update_state()
        fb.update_state()
        self.assertEqual(fb.voltage, 0.001)
        self.assertEqual(fb.current, 1.0)
        self.assertEqual(fb.power_supply_status, 1)

    def test_discharging_state(self):
        ''' Test discharing state of battery after fully charged '''

        fb: FakeBattery = FakeBattery(capacity=1.0, rate=1)
        # Setting battery to Fully charged status
        fb.power_supply_status = 4
        fb.voltage = 1.0
        fb.update_state()

        # Voltage will start going down
        self.assertTrue(fb.voltage < 1.0)

        # Current should be negative
        self.assertTrue(fb.current < 0)

        # Discharging status
        self.assertEqual(fb.power_supply_status, 2)

    def test_charging_state(self):
        ''' Test charging state of battery after fully discharged '''

        fb: FakeBattery = FakeBattery(capacity=1.0, rate=1)
        # Setting battery to Not charged status
        fb.power_supply_status = 3
        fb.voltage = 0
        fb.update_state()

        # Voltage will start going up
        self.assertTrue(fb.voltage > 0)

        # Current should be 1.0
        self.assertEqual(fb.current, 1.0)

        # Charging status
        self.assertEqual(fb.power_supply_status, 1)


if __name__ == '__main__':
    unittest.main()
