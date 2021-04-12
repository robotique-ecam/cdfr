![Simulation](https://github.com/3wnbr1/ros/workflows/Simulation/badge.svg)
![ROS2](https://img.shields.io/badge/ros2-foxy-blue)
![Webots](https://img.shields.io/badge/webots-2021a-blue)
![Status](https://img.shields.io/badge/status-beta-blueviolet)
![Version](https://img.shields.io/badge/version-v0.9-blue)

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
ros2 action send_goal /asterix/navigate_to_pose nav2_msgs/action/NavigateToPose "{'pose': {'pose': {'position': {'x': 1.25, 'y': 0.25}, 'orientation':{'z':0.7071068, 'w':0.7071068}}}}"
```

##### Deploy pharaon

```
ros2 service call /pharaon/deploy std_srvs/srv/Trigger
```

##### Disable Aruco detection

```
ros2 service call /enable_aruco_detection std_srvs/SetBool "{data: false}"
```

##### Publish to LCD screen

```
ros2 topic pub -1 /obelix/lcd lcd_msgs/Lcd "{text: 'sudo reboot', line: 1}"
```


### Changelog

#### v0.9.0
- Improved overall documentation rules
- Fixed map_server launcher [#44](https://github.com/robotique-ecam/cdfr/issues/44)
- Improved odometry readjustment routine [#43](https://github.com/robotique-ecam/cdfr/pull/43)
- Added Vlx odometry readjustment functionality [#43](https://github.com/robotique-ecam/cdfr/pull/43)
- Added odom_map_relative Pose_Stamped topic [#43](https://github.com/robotique-ecam/cdfr/pull/43)
- Refactored Localisation [#43](https://github.com/robotique-ecam/cdfr/pull/43)
- Added vlx_manager in Webots [#43](https://github.com/robotique-ecam/cdfr/pull/43)
- Added 2 teb_controllers (nominal, accurate) [#36](https://github.com/robotique-ecam/cdfr/pull/36)
- Added custom_goal_checker plugin [#36](https://github.com/robotique-ecam/cdfr/pull/36)
- Added costmap_converter_custom plugin [#36](https://github.com/robotique-ecam/cdfr/pull/36)
- Added bt_boolean_condition plugin [#36](https://github.com/robotique-ecam/cdfr/pull/36)
- Added bt_split_goal plugin [#36](https://github.com/robotique-ecam/cdfr/pull/36)
- Added costmap_converter_custom plugin [#35](https://github.com/robotique-ecam/cdfr/pull/35)
- Added black python code formatter check in CI [#32](https://github.com/robotique-ecam/cdfr/pull/32)
- Added service adjust_odometry [#31](https://github.com/robotique-ecam/cdfr/pull/31)
- Added possibility to readjust odometry [#31](https://github.com/robotique-ecam/cdfr/pull/31)
- Improved teb_controller parameters [#29](https://github.com/robotique-ecam/cdfr/pull/29)
- Improved navigation2 parameters [#29](https://github.com/robotique-ecam/cdfr/pull/29)

#### v0.8.4
- Added localisation node compatible with side selection interfaces with service call for odometry correction
- Fixed bug in costmap causing robot position to be evaluated outside of it
- Fixed bugs in NavigateToPose calculation
- Added actuators drivers with full support for ArbotixM
- Added templating for YAML files
- Added non-blocking selftest upon startup
- Added service call disable for aruco detection
- Added service call disable for drive steppers
- Added actuators logic in strategy
- Fixed ATTiny-85 firmwares for sliders
- Improved assurancetourix aruco detection with better camera calibration
- Improved and fixed ATTiny for steppers *WARNING : There is still an unresolved bug which causes I2C bus to freeze randomly*
- Fixed VL53L1X sensors initialisation board's firmware
- Refactored Pharaon firmware
- Refactored Panoramix
- Refactored cetautomatix
- Removed sensors node
- Droped ubuntu 20.04.1 LTS for Raspbian Buster 10
- Precompiled ros2 binaries now available in releases


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
