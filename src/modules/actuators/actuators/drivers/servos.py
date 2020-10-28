#!/usr/bin/env python3


"""Sliders Hardware Interface."""


class I2CServosDriver:
    """Driver interacting over I2C with sliders."""

    def __init__(self, i2c, addr=0x12):
        """Create driver with default values."""
        self._i2c = i2c
