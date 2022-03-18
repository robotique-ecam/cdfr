# Localisation package

## What does this package do ?

- Publish the initial map->odom transformation,
- Subscribe to "odom" topic and publish on "odom_map_relative" the map relative robot pose,
- Compute the transformation to re-adjust odometry and publish it.


## Odometry re-adjustment method

In order to re-adjust the odometry, 1 thing is used:
- Vision from assurancetourix package,

### Vision

Assurancetourix is publishing aruco positions and orientation through /allies_positions_markers topic. By subscribing to the topic and identify the marker that correspond to the node's namespace, the position and orientation of the robot can be found.
