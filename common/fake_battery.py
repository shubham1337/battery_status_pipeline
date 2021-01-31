
import math
import random
from random import gauss


class FakeBattery:
    ''' 
        A class which tries to imitate voltage, current and power_supply_status values 
        for a battery which is constantly going through the following states:
        Not charged -> Charging -> Fully charged -> Discharging -> Not charged
    '''

    def __init__(self, capacity: float = 1.0, rate: int = 1) -> None:
        self.max_capacity: float = capacity # Volts
        self.rate: float = 1.0 / rate # times per second
        self.voltage: float = 0.0 # Volts
        self.current: float = 0.0 # Ampere
        self.power_supply_status: int = 2

    def update_state(self) -> None:
        ''' This function is called to update the state(current, voltage, power_supply_status) of the battery '''

        # Charging or Started charging after exhausting all charge
        if self.power_supply_status == 1 or self.power_supply_status == 3:
            self.voltage += 0.001 * self.rate
            self.current = 1.0
            self.power_supply_status = 1

            # Reset battery after it has been fully charged
            if self.voltage >= self.max_capacity:
                self.voltage = self.max_capacity
                self.power_supply_status = 4

        # Discharging or Started discharing after Fully Charged
        elif self.power_supply_status == 2 or self.power_supply_status == 4:
            self.voltage -= 0.01 * self.rate

            # Current follows a gaussian distribution with mean at 0.25 and variance of 0.8 while discharging
            self.current = -abs(gauss(0.25, math.sqrt(0.8)))

            # Generates random current spikes of 4A while discharging
            if random.random() > 0.9:
                self.current = -4.0

            # Sets battery status to Discharging
            self.power_supply_status = 2

            # Reset battery after it is completely discharged
            if self.voltage <= 0:
                self.voltage = 0
                self.power_supply_status = 3
