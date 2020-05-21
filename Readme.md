![Simulation](https://github.com/3wnbr1/ros/workflows/Simulation/badge.svg)
![ROS2](https://img.shields.io/badge/ros2-foxy-blue)
![Webots](https://img.shields.io/badge/webots-2020a%20r2-blue)
![Status](https://img.shields.io/badge/status-alpha-blueviolet)
![Version](https://img.shields.io/badge/version-v0.8.0-blue)

### Build

#### Onboard build

```bash
cd ~/ros
. /opt/ros/eloquent/setup.bash
./setup.bash robot_name
. install/setup.bash
```

#### Dev / Simulation build

```bash
. /path/to/ros/install/setup.bash
cd path/to/ros
./setup.bash simulation
. install/setup.bash
```


### Changelog

#### v0.7. 0
- Full support for asterix, obelix, assurancetourix and pharaon running simultaneously
- Upstreamed navigation2 stack to master branch
- Refactored behavior trees with full strategy engine support
- Robot launchers templating
- Support for border in map
- Added transformix service for transformations
- Added firmware for microcontrolers and pharaon
- Consistent code formatting through the whole repository
- Refactored setup script and updated docs
- Default build type for cmake is now Release
- Updated CI to use setup script in simulation mode
- Added changelog


#### v0.6.0b
- Added actuators node, with full support for Arbotix-M and actuators board
- Partial namespacing for asterix
- Assurancetourix node fine tuning
- Added cetautomatix node, descision node based on behavior trees
- Added ST VL53L1X ToF sensors support in sensor node
- Creation of docs for I2C and debugging
- Created panoramix server node


#### v0.5.0
- Pharaon interface as ROS 2 service
- URDF XACRO templating for asterix and obelix
- Added support for MIPI_CAMERA in assurancetourix
- Higher accuracy in navigation


#### v0.4.0
- Created assurancetourix ARUCO detection Node
- Added LCD screen driver node
- Tuning the navigation stack


#### v0.3.0
- Fixes inside drive node for odometry
- Added sensors inside URDF
- Adding support for ROS 2 navigation stack


#### v0.2.0
- Adding odometry to drive node
- Adding sensors node with support for custom HCSR04 over I2C
- Adding parameters file


#### v0.1.0
- Drive node creation. Supports receiving speed commands
- Range sensors PoC using ARM mbed on FRDM K64F
