# period number based on a 48MHz clock
periods = [
    959000,
    957000,
    953000,
    949000,
    947000,
    943000,
    941000,
    939000,
    937000,
    929000,
    919000,
    911000,
    907000,
    901000,
    893000,
    887000,
]

data_frequency = 6e6

dist_from_center = 0.05
dist_between_sensors = 0.04
sensor_id_to_position = [
    (-dist_between_sensors / 2, dist_from_center, 0.0),
    (dist_between_sensors / 2, dist_from_center, 0.0),
    (dist_from_center, dist_between_sensors / 2, 0.0),
    (dist_from_center, -dist_between_sensors / 2, 0.0),
    (dist_between_sensors / 2, -dist_from_center, 0.0),
    (-dist_between_sensors / 2, -dist_from_center, 0.0),
    (-dist_from_center, -dist_between_sensors / 2, 0.0),
    (-dist_from_center, dist_between_sensors / 2, 0.0),
]
