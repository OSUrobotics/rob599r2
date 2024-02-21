cmake_minimum_required(VERSION 3.8)
project(rob599_basic_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set your compile options here.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the relevant dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rob599_msgs REQUIRED)

# List your executables and their source files here.
add_executable(node src/node.cpp)
add_executable(counter src/counter.cpp)
add_executable(doubler src/doubler.cpp)
add_executable(params src/params.cpp)
add_executable(fib_server src/fib_server.cpp)
add_executable(fib_client src/fib_client.cpp)
add_executable(fib_client_2 src/fib_client_2.cpp)
#add_executable(fib_action_server src/fib_action_server.cpp)
#add_executable(fib_action_client src/fib_action_client.cpp)
add_executable(cloud_generator src/cloud_generator.cpp src/cloud_generator_node.cpp)
add_executable(cloud_timer src/cloud_timer.cpp src/cloud_timer_node.cpp)
add_executable(cloud_combined src/cloud_combined.cpp src/cloud_generator_node.cpp src/cloud_timer_node.cpp)

# List the executable dependencies here.
ament_target_dependencies(node rclcpp)
ament_target_dependencies(counter rclcpp std_msgs)
ament_target_dependencies(doubler rclcpp std_msgs)
ament_target_dependencies(params rclcpp std_msgs)
ament_target_dependencies(fib_server rclcpp std_msgs rob599_msgs)
ament_target_dependencies(fib_client rclcpp std_msgs rob599_msgs)
ament_target_dependencies(fib_client_2 rclcpp std_msgs rob599_msgs)
#ament_target_dependencies(fib_action_server rclcpp std_msgs rob599_msgs rclcpp_action)
#ament_target_dependencies(fib_action_client rclcpp std_msgs rob599_msgs rclcpp_action)
ament_target_dependencies(cloud_generator rclcpp sensor_msgs)
ament_target_dependencies(cloud_timer rclcpp std_msgs sensor_msgs)
ament_target_dependencies(cloud_combined rclcpp std_msgs sensor_msgs)

# Where do we want these to be installed?
install(TARGETS
  node
  counter
  doubler
  params
  fib_server
  fib_client
  fib_client_2
#  fib_action_server
#  fib_action_client
  cloud_generator
  cloud_timer
  cloud_combined
  DESTINATION lib/${PROJECT_NAME}
)

# Make sure the launch files make it to the right place
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
