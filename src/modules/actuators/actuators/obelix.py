#!/usr/bin/env python3


"""Actuator definition for robot Obelix."""


from actuators.actuators import NC, NO, Actuators


DYNA_UP = 290
DYNA_DOWN = 380
DYNA_SPEED = 120


# Pumps and valves adresses
PUMPS = {
    1: {'pump': 0, 'valve': 1, 'type': NC, 'pos': (8, -11)},
    2: {'pump': 2, 'valve': 3, 'type': NC, 'pos': (8, -4)},
    3: {'pump': 4, 'valve': 9, 'type': NC, 'pos': (8, 4)},
    4: {'pump': 10, 'valve': 11, 'type': NC, 'pos': (8, 11)},
    5: {'pump': 0, 'valve': 1, 'type': NO},
    6: {'pump': 2, 'valve': 3, 'type': NO},
    7: {'pump': 4, 'valve': 9, 'type': NO},
    8: {'pump': 10, 'valve': 11, 'type': NO},
    9: {'pump': 12, 'valve': 13, 'type': NO},
}

# Fans addresses
FANS = [7, 15]

# Servo
SERVOS = {
    'flags': {'addr': 5, 'down': 485, 'up': 205, 'speed': 250}
}

# Dynamixels
DYNAMIXELS = {
    1: {'speed': DYNA_SPEED, 'up': DYNA_UP, 'down': DYNA_DOWN},
    2: {'speed': DYNA_SPEED, 'up': DYNA_UP, 'down': DYNA_DOWN},
    3: {'speed': DYNA_SPEED, 'up': DYNA_UP, 'down': DYNA_DOWN},
    4: {'speed': DYNA_SPEED, 'up': DYNA_UP, 'down': DYNA_DOWN},
}

actuators = Actuators(
    i2c_bus=5,
    pump_addr=[0x40, 0x41],
    FANS=FANS,
    PUMPS=PUMPS,
    SERVOS=SERVOS,
    DYNAMIXELS=DYNAMIXELS
)
