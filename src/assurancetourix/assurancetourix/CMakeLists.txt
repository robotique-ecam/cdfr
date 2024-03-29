cmake_minimum_required(VERSION 3.5)
project(assurancetourix)

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
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(transformix_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "tf2"
  "rclcpp"
  "visualization_msgs"
  "OpenCV"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs"
  "transformix_msgs"
)

set(EXEC_NAME "assurancetourix")

add_executable(${EXEC_NAME}
  src/assurancetourix.cpp
  src/assurancetourix_node.cpp
  src/geometrix.cpp
)

option(CAMERA "Use camera" OFF)
option(SIMULATION "Simulation mode" OFF)

if(SIMULATION)
  add_definitions(-DSIMULATION)
  if(NOT DEFINED ENV{WEBOTS_HOME})
    message(FATAL_ERROR "WEBOTS_HOME is not set")
  endif()
  # Link with the Webots controller library.
  set (LIBRARIES $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)
  target_link_libraries(${EXEC_NAME} ${LIBRARIES})
endif()

if(CAMERA)
  add_definitions(-DCAMERA)
endif()

if(EXTRALOG)
  add_definitions(-DEXTRALOG)
endif()

ament_target_dependencies(${EXEC_NAME} ${dependencies})


################################################################################
# Install
################################################################################

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_package()
