cmake_minimum_required(VERSION 3.5)
project(map)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)


################################################################################
# Install
################################################################################
execute_process(COMMAND python3 map.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/map)

install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
