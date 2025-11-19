cmake_minimum_required(VERSION 3.8)
project(point_lio)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-O3")

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[point_lio]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
  pcl_ros
  mrs_lib
  backward_ros
)

set(LIBRARIES
  PointLIO_PointLIO
)

find_package(PCL 1.8 REQUIRED)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

add_library(PointLIO_PointLIO SHARED
  src/laser_mapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
  src/estimator.cpp
)

ament_target_dependencies(PointLIO_PointLIO
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(PointLIO_PointLIO PLUGIN "point_lio::PointLio" EXECUTABLE PointLIO_PointLIO)

target_compile_definitions(PointLIO_PointLIO PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
