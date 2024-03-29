cmake_minimum_required(VERSION 3.5)
project(sensor_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(sensor_pointcloud 
  src/sensor_pointcloud.cpp
  src/sensor.cpp
  src/sensor_precipitator.cpp)
ament_target_dependencies(sensor_pointcloud 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs )
  
target_include_directories(sensor_pointcloud PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  sensor_pointcloud
  DESTINATION lib/${PROJECT_NAME})

ament_package()
