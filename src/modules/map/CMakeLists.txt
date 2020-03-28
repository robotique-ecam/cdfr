cmake_minimum_required(VERSION 3.5)
project(map)


################################################################################
# Install
################################################################################
execute_process(COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/map/map.py ${CMAKE_CURRENT_SOURCE_DIR})

install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
