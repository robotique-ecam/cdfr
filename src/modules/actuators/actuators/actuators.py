#!/usr/bin/env python3


"""Shared utils functions between asterix and obelix."""


from time import sleep

from .arbotix.arbotix import ArbotiX
from .pumps.pumps import PumpDriver

NO, NC = False, True
DYNA_UP, DYNA_DOWN = 950, 800

DYNAMIXELS_SPEED = 100


class Actuators:

    """Actuators base class."""

    def __init__(self, i2c_bus=5, pump_addr=[0x40], FANS=[7], PUMPS={}, DYNAMIXELS=None, SERVOS={}):
        """."""
        self.FANS = FANS
        self.PUMPS = PUMPS
        self.SERVOS = SERVOS
        self.DYNAMIXELS = DYNAMIXELS
        self.pump_driver = PumpDriver(addrs=pump_addr, bus_id=i2c_bus)
        if DYNAMIXELS is not None:
            self.arbotix = ArbotiX()
            self._setupDynamixels()

    def _setupDynamixels(self):
        """Setup dynamixels speed."""
        for dyna in self.DYNAMIXELS:
            self.arbotix.setSpeed(dyna, DYNAMIXELS_SPEED)
            self.arbotix.setPosition(dyna, DYNA_DOWN)

    def raiseTheFlag(self):
        """Raise obelix flags. Servo must be bound with ArbotixM."""
        if flag_servo := self.SERVOS.get('flags') is not None:
            self.arbotix.setServo(flag_servo['addr'], flag_servo['high'])

    def setPumpsEnabled(self, enabled: bool, pumps: list):
        """Set list of pumps as enabled or not."""
        relax = []
        for p in pumps:
            pump = self.PUMPS.get(p)
            if pump is not None:
                if enabled and pump.get('type') == NC:
                    self.pump_driver.bytes_set([pump.get('pump')])
                elif enabled and pump.get('type') == NO:
                    self.pump_driver.bytes_set([pump.get('pump'), pump.get('valve')])
                elif not enabled and pump.get('type') == NC:
                    self.pump_driver.bytes_clear([pump.get('pump')])
                    self.pump_driver.bytes_set([pump.get('valve')])
                    relax.append(pump.get('valve'))
                    self.pump_driver.bytes_clear([pump.get('valve')])
                elif not enabled and pump.get('type') == NO:
                    self.pump_driver.bytes_clear([pump.get('pump'), pump.get('valve')])
        # Relax valves to normal state after 100ms minimum delay
        if len(relax) > 0:
            sleep(.1)
            self.pump_driver.bytes_clear(relax)

    def setDynamixelsPositions(self, addrs: list, positions: list):
        """Set list of pumps as enabled or not."""
        for addr, position in zip(addrs, positions):
            self.arbotix.setPosition(addr, position)

    def setFansEnabled(self, enabled: bool):
        """Set fans on and off."""
        if enabled:
            self.pump_driver.bytes_set(self.FANS)
        else:
            self.pump_driver.bytes_clear(self.FANS)
