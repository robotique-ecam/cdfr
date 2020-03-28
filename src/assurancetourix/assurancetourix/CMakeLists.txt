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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "tf2"
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "OpenCV"
  "geometry_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

set(EXEC_NAME "assurancetourix")


option(MIPI_CAMERA "Use MIPI camera" OFF)


add_executable(${EXEC_NAME}
  src/assurancetourix.cpp
  src/assurancetourix_node.cpp
)

if(MIPI_CAMERA)
  add_definitions(-DMIPI_CAMERA)
  target_link_libraries(${EXEC_NAME} -larducam_mipicamera)
endif()

ament_target_dependencies(${EXEC_NAME} ${dependencies})


################################################################################
# Install
################################################################################

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_package()
