cmake_minimum_required(VERSION 3.5)
project(obelix)


################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)


################################################################################
# Install
################################################################################

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
