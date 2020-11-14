#!/usr/bin/env python3


"""Sliders Hardware Interface."""


try:
    from gpiozero import Servo
except ImportError:
    print('[!] RPI Servos are in simulation mode. python3-gpiozero was found !')


class RPiServos:
    """Driver interacting with Hobby servos plugged onto PWM GPIOs."""

    def __init__(self, pins={}):
        """Create driver with default values."""
        self.servos = {}
        try:
            for pin in pins:
                self.servos.update({pin: Servo(pin)})
        except NameError:
            pass

    def set_angles(self, pins, values):
        """Set servo angles."""
        try:
            for pin, val in zip(pins, values):
                self.servos[pin].value = val
        except NameError:
            print(f'Servo {pin} set to {val}', flush=True)
