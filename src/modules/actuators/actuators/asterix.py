#!/usr/bin/env python3


"""Actuator definition for robot Asterix."""

from actuators.actuators import NC, NO, Actuators

# Pumps and valves adresses
PUMPS = {
    1: {
        "pump": 14,
        "valve": 15,
        "push": 7,
        "pull": 6,
        "type": NC,
        "pos": (-0.089, -0.105),
    },
    2: {
        "pump": 10,
        "valve": 11,
        "push": 5,
        "pull": 4,
        "type": NC,
        "pos": (0.0, -0.105),
    },
    3: {
        "pump": 8,
        "valve": 9,
        "push": 1,
        "pull": 0,
        "type": NC,
        "pos": (0.089, -0.105),
    },
}

# Servo
SERVO_MOTORS = {
    "resistance": 6
}

actuators = Actuators(
    i2c_bus=3,
    pump_addr=[0x41, 0x40],
    PUMPS=PUMPS,
    SERVO_MOTORS=SERVO_MOTORS
)