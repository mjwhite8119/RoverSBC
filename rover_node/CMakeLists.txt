cmake_minimum_required(VERSION 3.5)
project(rover_node)

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

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/diff_drive_controller.cpp"
  "src/odometry.cpp"
  "src/rover.cpp"
)

set(DEPENDENCIES
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "rover_ros")
set(BROADCASTER_NAME "rover_tf2_broadcaster")

add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

add_executable(${BROADCASTER_NAME} src/rover_tf2_broadcaster.cpp)
ament_target_dependencies(${BROADCASTER_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  ${EXECUTABLE_NAME} 
  ${BROADCASTER_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
