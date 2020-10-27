#!/usr/bin/env python3


"""Sliders Hardware Interface."""


from time import sleep

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
except ImportError:
    print('[!] RPI Servos are in simulation mode. No RPi.GPIO was found !')
    SMBus = int


class RPiServos:
    """Driver interacting with Hobby servos plugged onto PWM GPIOs."""

    def __init__(self, pins=[]):
        """Create driver with default values."""
        self.PWM_FREQ = 5
        self._pins = {}
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            self._pwms.update({pin: GPIO.PWM(pin, self.PWM_FREQ)})
            self._pwms[pin].start(0)

    def _set_angle(self, pin, value):
        """Set servo angle in degrees at BCM pin."""
        if pin in self._pwms:
            duty_c = value / 18 + 2
            GPIO.setup(pin, GPIO.OUT)
            self._pwms[pin].ChangeDutyCycle(duty_c)

    def _relax(self, pin):
        """Set servo angle in degrees at BCM pin."""
        if pin in self._pwms:
            GPIO.setup(pin, False)
            self._pwms[pin].ChangeDutyCycle(0)

    def set_angles(self, pins, values):
        for pin, val in zip(pins, values):
            self._set_angle(pin, val)
        sleep(1)
        for pin in pins:
            self._relax(pin)
