cmake_minimum_required(VERSION 3.5)
project(rover_msgs)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/SensorState.msg"
#  "msg/Sound.msg"
#  "msg/VersionInfo.msg"
)

#set(srv_files
#  "srv/Sound.srv"
#  "srv/Dqn.srv"
#)

#set(action_files
#  "action/Patrol.action"
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
#  ${srv_files}
#  ${action_files}
#  DEPENDENCIES action_msgs std_msgs
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()