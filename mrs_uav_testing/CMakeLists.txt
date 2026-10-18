cmake_minimum_required(VERSION 3.28)
project(mrs_uav_testing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  mrs_lib
  mrs_msgs
  tf2_ros
  backward_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(MrsUavTesting_MrsUavTesting)
add_library(mrs_uav_testing::mrs_uav_testing ALIAS MrsUavTesting_MrsUavTesting)
set_target_properties(MrsUavTesting_MrsUavTesting
  PROPERTIES
    EXPORT_NAME mrs_uav_testing
    OUTPUT_NAME mrs_uav_testing
)

target_sources(MrsUavTesting_MrsUavTesting
  PRIVATE
    "src/test_generic.cpp"
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_uav_testing/test_generic.h"
)

target_link_libraries(MrsUavTesting_MrsUavTesting
  PUBLIC
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    tf2_ros::tf2_ros
    ${mrs_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  if(NOT DEFINED ENV{IN_NIX_SHELL})
    message(WARNING "Compiling lint tests")
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint()
  endif()

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS MrsUavTesting_MrsUavTesting
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  MrsUavTesting_MrsUavTesting
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
