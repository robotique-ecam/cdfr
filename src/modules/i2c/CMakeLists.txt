cmake_minimum_required(VERSION 3.5)
project(i2c)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################################################
# Build
################################################################################

add_library(${PROJECT_NAME}
  include/i2c.hpp
  src/i2c.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/i2c.hpp)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
