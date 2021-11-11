#!/usr/bin/env python3


"""Actuator definition for robot Obelix."""


from actuators.actuators import NC, NO, Actuators


DYNA_UP = 290
DYNA_DOWN = 380
DYNA_SPEED = 120


# Pumps and valves adresses
PUMPS = {
    1: {"pump": 6, "valve": 0, "type": NC, "pos": (0.09, -0.115)},
    2: {"pump": 5, "valve": 0, "type": NC, "pos": (0.09, -0.0375)},
    3: {"pump": 4, "valve": 0, "type": NC, "pos": (0.09, 0.0375)},
    4: {"pump": 3, "valve": 0, "type": NC, "pos": (0.09, 0.115)},
    # 5: {"pump": 7, "valve": 0, "type": NO, "pos": (0, -0.1)},
    # 6: {"pump": 2, "valve": 3, "type": NO, "pos": (0, -0.1)},
    # 7: {"pump": 4, "valve": 9, "type": NO, "pos": (0, -0.1)},
    # 8: {"pump": 10, "valve": 11, "type": NO, "pos": (0, -0.1)},
    # 9: {"pump": 12, "valve": 13, "type": NO, "pos": (0, -0.1)},
}

# Servo
SERVOS = {"flags": {"addr": 5, "down": 485, "up": 205, "speed": 250}}

# Dynamixels
DYNAMIXELS = {
    1: {"speed": DYNA_SPEED, "up": DYNA_UP, "down": DYNA_DOWN},
    2: {"speed": DYNA_SPEED, "up": DYNA_UP, "down": DYNA_DOWN},
    3: {"speed": DYNA_SPEED, "up": DYNA_UP, "down": DYNA_DOWN},
    4: {"speed": DYNA_SPEED, "up": DYNA_UP, "down": DYNA_DOWN},
}

SERVO_MOTORS = {
    "arm": 0x06,
    "reef": 0x06,
}

actuators = Actuators(
    i2c_bus=3,
    pump_addr=[0x40],
    PUMPS=PUMPS,
    SERVOS=SERVOS,
    DYNAMIXELS=DYNAMIXELS,
    i2c_bus_servo_motor=5,
    SERVO_MOTORS=SERVO_MOTORS,
)
