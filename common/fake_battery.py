
import math
import random
from random import gauss

class FakeBattery():

    def __init__(self, capacity=1, rate=1):
        self.max_capacity = capacity # V
        self.capacity = capacity # V
        self.rate = 1 / rate
        self.voltage = 0 # V
        self.current = 0 # A
        self.power_supply_status = 2

    def update_state(self):
        # Charging or Started charging after exhausting all charge
        if self.power_supply_status == 1 or self.power_supply_status == 3:
            self.voltage += 0.001
            self.current = 1
            self.capacity += self.voltage * self.rate
            if self.capacity >= self.max_capacity:
                self.capacity = self.max_capacity
                self.power_supply_status = 4
        # Discharging or Started discharing after Fully Charged
        elif self.power_supply_status == 2 or self.power_supply_status == 4:
            self.voltage += 0.01
            self.current = -abs(gauss(0.25, math.sqrt(0.8)))
            if random.random() > 0.9:
                self.current = -4.0
            self.capacity += self.voltage * self.rate
            if self.capacity <= 0:
                self.capacity = 0
                self.power_supply_status = 3
