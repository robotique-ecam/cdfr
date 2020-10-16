#!/usr/bin/env python3


"""Compute robot kinematics parameters to be inserted into YAMLs templates."""


from math import pi

import yaml


def get_footprint(length=.20, width=.15, wheels_excentration=0):
    """Get footprint corresponding to robot specified dimensions."""
    l, w, x = length / 2, width / 2, wheels_excentration
    f = [
        [-l + x, -w],
        [-l + x, w],
        [l + x, w],
        [l + x, -w],
    ]
    return str(f)


def compute_speeds(config: dict):
    """Compute kinemtics limits for each robot."""
    step = config.get('max_steps_frequency') / config.get('speedramp_resolution')
    steps_per_turn = config.get('steps_per_turn') * config.get('microsteps')
    meters_per_turn = 2 * pi * config.get('wheel_radius')
    max_turns_per_sec = config.get('max_steps_frequency') / steps_per_turn
    max_linear_vel = max_turns_per_sec * meters_per_turn
    max_linear_vel = round(max_linear_vel, 4)
    config['max_vel_x'] = max_linear_vel
    config['max_vel_x_backwards'] = max_linear_vel
    return config


globals = {
    'use_sim_time': False,

    'max_steps_frequency': 10000,
    'speedramp_resolution': 128,
    'microsteps': 16,

    'min_turning_radius': 0.0
}

asterix = {
    'wheel_separation': .244,
    'wheel_radius': .0405,
    'steps_per_turn': 200,
    'footprint': get_footprint(length=.19, width=.256),

    'max_vel_x': 0.4,
    'max_vel_x_backwards': 0.4,
    'max_vel_theta': 1.0,
    'acc_lim_x': 0.5,
    'acc_lim_theta': 0.5,
}

obelix = {
    'wheel_separation': .282,
    'wheel_radius': .061,
    'steps_per_turn': 400,
    'footprint': get_footprint(length=.18, width=.35),

    'max_vel_x': 0.4,
    'max_vel_x_backwards': 0.4,
    'max_vel_theta': 1.0,
    'acc_lim_x': 0.5,
    'acc_lim_theta': 0.5,
}

obelix.update(globals)
obelix = compute_speeds(obelix)
asterix.update(globals)
asterix = compute_speeds(asterix)

with open('obelix.vars', 'w') as f:
    f.write(yaml.dump(obelix))

with open('asterix.vars', 'w') as f:
    f.write(yaml.dump(asterix))
