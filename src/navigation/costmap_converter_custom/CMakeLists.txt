cmake_minimum_required(VERSION 3.5)
project(costmap_converter_custom)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    class_loader
    costmap_converter
    costmap_converter_msgs
    cv_bridge
    geometry_msgs
    nav2_costmap_2d
    tf2
    tf2_geometry_msgs
    tf2_ros
    OpenCV
    pluginlib
    rclcpp
)

# === Build ===

add_library(${lib_name} SHARED
            src/costmap_to_polygons.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
   DESTINATION lib
)

pluginlib_export_plugin_description_file(costmap_converter costmap_converter_plugins.xml)
ament_target_dependencies(${lib_name} ${dependencies})
ament_package()
