#!/usr/bin/env python3


"""Sliders Hardware Interface."""


class I2CSliderDriver:
    """Driver interacting over I2C with sliders."""

    def __init__(self, i2c, addr=0x12):
        """Create driver with default values."""
        self._i2c = i2c
        self._addr = addr

    def set_position(self, position: int):
        """Set the position of the slider"""
        self._i2c.write_byte(self._addr, 1 << 4 | position)
