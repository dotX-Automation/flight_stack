cmake_minimum_required(VERSION 3.8)
project(flight_control)

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
find_package(dua_app_management REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(params_manager REQUIRED)
find_package(pose_kit REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "{CMAKE_CURRENT_SOURCE_DIR}/src/flight_control/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure Flight Control node source files
file(GLOB FLIGHT_CONTROL_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/flight_control/*.cpp")
list(APPEND FLIGHT_CONTROL_NODE_SRCS init_parameters.cpp)

# Flight Control standalone application
add_executable(flight_control_app
  ${FLIGHT_CONTROL_NODE_SRCS}
  src/flight_control_app.cpp)
target_include_directories(flight_control_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_control_app
  Eigen3::Eigen
  Threads::Threads)
ament_target_dependencies(flight_control_app
  dua_app_management
  dua_interfaces
  dua_node
  dua_qos
  geometry_msgs
  message_filters
  params_manager
  pose_kit
  px4_msgs
  rclcpp
  rclcpp_action
  std_msgs)

# Install Flight Control app
install(TARGETS flight_control_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
