cmake_minimum_required(VERSION 3.5)
project(onboard_vision)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "rclcpp"
  "OpenCV"
)

set(EXEC_NAME "onboard_vision")


add_executable(${EXEC_NAME}
  src/onboard_vision_node.cpp
  src/onboard_vision.cpp
)

option(SIMULATION "Simulation mode" OFF)

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
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_msgs)
ament_package()
