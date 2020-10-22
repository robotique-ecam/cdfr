cmake_minimum_required(VERSION 3.5)
project(drive)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
  "std_srvs"
  "diagnostic_msgs"
)

set(EXEC_NAME "drive")


add_executable(${EXEC_NAME}
  src/drive_node.cpp
  src/drive.cpp
  src/i2c.cpp
)

option(DEBUG "Debug mode" OFF)
option(SIMULATION "Simulation mode" OFF)
option(TIMER "Enable Wall Timer for odometry" ON)

if(SIMULATION)
  add_definitions(-DSIMULATION)
  if(NOT DEFINED ENV{WEBOTS_HOME})
    message(FATAL_ERROR "WEBOTS_HOME is not set")
  endif()
  # Link with the Webots controller library.
  set (LIBRARIES $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)
  target_link_libraries(${EXEC_NAME} ${LIBRARIES})
else()
  if (DEBUG)
    add_definitions(-DDEBUG)
  endif()
  target_link_libraries(${EXEC_NAME} i2c)
endif()


if(TIMER)
  add_definitions(-DUSE_TIMER)
endif()

ament_target_dependencies(${EXEC_NAME} ${dependencies})


################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
