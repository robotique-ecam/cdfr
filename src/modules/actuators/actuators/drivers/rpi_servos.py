#!/usr/bin/env python3


"""Servo motor Hardware Interface."""


class RPiServos:
    """Driver interacting with Hobby servos plugged onto an external arduino nano."""

    def __init__(self, i2c_bus_servo, addrs={}):
        """Create driver with default values."""
        self.i2c = i2c_bus_servo
        self.servos_addrs = addrs

    def set_angles(self, servo: str, value: int):
        """Set servo angles. servo in ["arm", "reef"]
        reef: 0 --> closed, 37 --> opened
        arm: 82 --> closed, 45 --> opened
        value between [0, 90]"""
        if servo == "arm":
            self.i2c.write_byte(self.servos_addrs[servo], value)
        elif servo == "reef":
            self.i2c.write_byte(self.servos_addrs[servo], 1 << 7 | value)
        else:
            self.i2c.write_byte(self.servos_addrs[servo], value)
            
    def read_value(self, servo: str) -> int:
        return self.i2c.read_byte(self.servos_addrs[servo])
