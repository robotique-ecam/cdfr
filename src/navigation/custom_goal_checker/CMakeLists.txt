cmake_minimum_required(VERSION 3.5)
project(custom_goal_checker)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  angles
  rclcpp
  std_msgs
  nav2_util
  nav2_core
  pluginlib
)

nav2_package()

# === Build ===

include_directories(include)
add_library(${lib_name} SHARED src/custom_goal_checker.cpp)

# === Installation ===

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_core custom_goal_checker_plugin.xml)
ament_target_dependencies(${lib_name} ${dependencies})
target_compile_definitions(${lib_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_package()
