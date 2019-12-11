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
include_directories(
  include
)

set(EXEC_NAME "i2c")

add_library(${PROJECT_NAME}
  src/i2c.cpp
)

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  PUBLIC_HEADER DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
# ament_package()
