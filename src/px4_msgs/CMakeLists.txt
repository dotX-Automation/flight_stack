cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# Generate introspection typesupport for C and C++ and IDL files for ROS 2
rosidl_generate_interfaces(${PROJECT_NAME}
  ${PX4_MSGS}
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS)

ament_package()
