#!/usr/bin/env python3


"""Actuator definition for robot Asterix."""


from actuators.actuators import Actuators


actuators = Actuators(pump_addr=[0x40, 0x41])
