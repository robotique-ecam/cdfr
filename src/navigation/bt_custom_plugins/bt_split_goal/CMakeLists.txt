cmake_minimum_required(VERSION 3.5)
project(bt_split_goal)

set(lib_name ${PROJECT_NAME})

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
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  behaviortree_cpp_v3
  tf2_ros
  nav2_util
  nav2_costmap_2d
)

# === Build ===

include_directories(include)

add_library(${lib_name} SHARED
  src/split_goal_action.cpp
)

# === Installation ===

ament_target_dependencies(${lib_name} ${dependencies})
target_compile_definitions(${lib_name} PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES bt_split_goal_tree_node.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dependencies})
ament_package()
