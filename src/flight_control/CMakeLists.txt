cmake_minimum_required(VERSION 3.15)
project(flight_control)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

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

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_signal_handler REQUIRED)
find_package(stanis_interfaces REQUIRED)
find_package(stanis_qos REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Flight Control standalone application
add_executable(flight_control
  src/flight_control/flight_control.cpp
  src/flight_control/fc_actions.cpp
  src/flight_control/fc_flight_routines.cpp
  src/flight_control/fc_services.cpp
  src/flight_control/fc_subscriptions.cpp
  src/flight_control/fc_timers.cpp
  src/flight_control/fc_utils.cpp
  src/flight_control_app.cpp)
target_include_directories(flight_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_control Eigen3::Eigen Threads::Threads)
ament_target_dependencies(
  flight_control
  "geometry_msgs"
  "message_filters"
  "px4_msgs"
  "rclcpp"
  "rclcpp_action"
  "ros2_signal_handler"
  "stanis_interfaces"
  "stanis_qos"
  "std_msgs"
)

# flight_control app
install(TARGETS flight_control
  DESTINATION lib/${PROJECT_NAME})

# Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch files
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
