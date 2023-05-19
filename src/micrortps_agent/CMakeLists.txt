cmake_minimum_required(VERSION 3.8)
project(micrortps_agent)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(params_manager REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Determine and find RMW implementation
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

# Find Python 3 installation
find_package(PythonInterp 3 REQUIRED)

# Generate microRTPS Agent source code
get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
set(MSGS_DIR "${PX4_MSGS_DIR}/msg" CACHE INTERNAL "MSGS_DIR")
include(GenerateMicroRTPSAgent)

# Build Transport library
add_library(rtpstransport STATIC
  src/transport/transport.cpp)
target_compile_features(rtpstransport PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(rtpstransport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rtpstransport
  rclcpp)

# Build Agent app
add_executable(micrortps_agent_app
  ${MICRORTPS_AGENT_APP_FILES})
target_include_directories(micrortps_agent_app PUBLIC
  $<BUILD_INTERFACE:${MICRORTPS_AGENT_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(micrortps_agent_app PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_definitions(micrortps_agent_app PRIVATE "BUILDING_AGENT")
target_link_libraries(micrortps_agent_app
  fastcdr
  fastrtps
  rtpstransport
  Threads::Threads)
ament_target_dependencies(micrortps_agent_app
  builtin_interfaces
  dua_app_management
  dua_node
  params_manager
  px4_msgs
  rclcpp
  sensor_msgs
  std_msgs)

# Install Agent app
install(TARGETS micrortps_agent_app
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
