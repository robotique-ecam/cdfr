![Simulation](https://github.com/3wnbr1/ros/workflows/Simulation/badge.svg)
![ROS2](https://img.shields.io/badge/ros2-foxy-blue)
![Webots](https://img.shields.io/badge/webots-2020b-blue)
![Status](https://img.shields.io/badge/status-beta-blueviolet)
![Version](https://img.shields.io/badge/version-v0.8.3-blue)

### Build

#### Onboard build

```bash
cd ~/ros
source /opt/ros/foxy/setup.bash
./setup.bash robot_name
source install/setup.bash
```

#### Dev / Simulation build

```bash
. /path/to/ros/install/setup.bash
cd path/to/ros
./setup.bash simulation
. install/setup.bash
```


### Few service calls

##### Enable stepper drivers

```
ros2 service call /obelix/enable_drivers  std_srvs/SetBool "{data: true}"
```

##### Send action goal

```
ros2 action send_goal /obelix/navigate_to_pose nav2_msgs/action/NavigateToPose "{'pose': {'pose': {'position': {'x': 1, 'y': 1}}}}"
```

##### Deploy pharaon

```
ros2 service call /pharaon/deploy std_srvs/srv/Trigger
```

##### Disable Aruco detection

```
ros2 service call /enable_aruco_detection std_srvs/SetBool "{data: false}"
```

### Changelog

#### v0.8.0
- Updated linux distro to ubuntu 20.04 LTS
- Updated ROS2 distro to Foxy
- Added webots simulation world for Physical based simulation
- Automated cloud based simulation on Github Actions CI
- New Timed Elastic Band (TEB) Controller achieving high speed and precision
- Added package Titan for simulation mode
- Compatibility with ros2_cross_compile for packaging x86_64/arm64 ros2 packages
- Refactored cetautomatix
- Simulation mode for assurancetourix, drive and pharaon
- Fixed many bugs thanks to simulation mode, including race conditions between modules

#### v0.7.0
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
