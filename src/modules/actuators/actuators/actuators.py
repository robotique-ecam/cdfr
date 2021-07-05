#!/usr/bin/env python3


"""Shared utils functions between asterix and obelix."""


from time import sleep

from actuators.arbotix.arbotix import ArbotiX
from actuators.drivers.i2c import I2CDriver
from actuators.drivers.slider import I2CSliderDriver
from actuators.drivers.pumps import PumpDriver
from actuators.drivers.rpi_servos import RPiServos

NO, NC = False, True


class Actuators:

    """Actuators base class."""

    def __init__(
        self, i2c_bus=3, pump_addr=[0x40], FANS=[7], PUMPS={}, DYNAMIXELS=[], SERVOS={}
    ):
        """."""
        self.FANS = FANS
        self.PUMPS = PUMPS
        self.SERVOS = SERVOS
        self.DYNAMIXELS = DYNAMIXELS
        self.pump_addr = pump_addr
        self._i2c_bus = I2CDriver(i2c_bus)
        self.rpi_servos = RPiServos(pins=[19, 6])
        self.pump_driver = PumpDriver(self._i2c_bus, addrs=self.pump_addr)
        self.slider = I2CSliderDriver(self._i2c_bus)
        if len(self.DYNAMIXELS) > 0:
            self.arbotix = ArbotiX()
            self._setupDynamixels()

    def _setupDynamixels(self):
        """Setup dynamixels speed."""
        for dyna in self.DYNAMIXELS:
            d = self.DYNAMIXELS[dyna]
            self.arbotix.setSpeed(dyna, d.get("speed"))
            self.arbotix.setPosition(dyna, d.get("down"))
        for s in self.SERVOS:
            servo = self.SERVOS[s]
            self.arbotix.setSpeed(servo.get("addr"), servo.get("speed"))
            self.arbotix.setPosition(servo.get("addr"), servo.get("down"))

    def disableDynamixels(self):
        """Setup dynamixels speed."""
        for dyna in self.DYNAMIXELS:
            self.arbotix.disableTorque(dyna)
        for s in self.SERVOS:
            servo = self.SERVOS[s]
            self.arbotix.disableTorque(servo.get("addr"))

    def raiseTheFlag(self):
        """Raise obelix flags. Servo must be bound with ArbotixM."""
        flag_servo = self.SERVOS.get("flags")
        if flag_servo is not None:
            self.arbotix.setPosition(flag_servo["addr"], flag_servo["up"])

    def setPumpsEnabled(self, enabled: bool, pumps: list):
        """Set list of pumps as enabled or not."""
        relax = []
        for p in pumps:
            pump = self.PUMPS.get(p)
            if pump is not None:
                if enabled and pump.get("type") == NC:
                    self.pump_driver.bytes_set([pump.get("pump")])
                elif enabled and pump.get("type") == NO:
                    self.pump_driver.bytes_set([pump.get("pump"), pump.get("valve")])
                elif not enabled and pump.get("type") == NC:
                    self.pump_driver.bytes_clear([pump.get("pump")])
                    self.pump_driver.bytes_set([pump.get("valve")])
                    relax.append(pump.get("valve"))
                    self.pump_driver.bytes_clear([pump.get("valve")])
                elif not enabled and pump.get("type") == NO:
                    self.pump_driver.bytes_clear([pump.get("pump"), pump.get("valve")])
        # Relax valves to normal state after 100ms minimum delay
        if len(relax) > 0:
            sleep(0.1)
            self.pump_driver.bytes_clear(relax)

    def grabCups(self, ports: list):
        """Grab cups at specified indexes."""
        self.setPumpsEnabled(True, ports)
        sleep(0.5)
        positions, servos = [], []
        for servo in ports:
            if servo in self.DYNAMIXELS:
                servos.append(servo)
                positions.append(self.DYNAMIXELS[servo].get("up"))
        self.setDynamixelsPositions(servos, positions)

    def dropCups(self, ports: list):
        """Drop cups at specified indexes."""
        positions, servos = [], []
        for servo in ports:
            if servo in self.DYNAMIXELS:
                servos.append(servo)
                positions.append(self.DYNAMIXELS[servo].get("down"))
        self.setDynamixelsPositions(servos, positions)
        self.setPumpsEnabled(False, ports)

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

    def setSliderPosition(self, position: int):
        """Slider position"""
        self.slider.set_position(position)
