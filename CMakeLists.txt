cmake_minimum_required(VERSION 3.5)
# cmake_minimum_required(VERSION 3.0.2)

##### anis had the below line which gave me an error
# cmake_policy(SET CMP0148 OLD)
#https://github.com/pybind/pybind11/issues/4785
#####

project(stair_modeling_ros)

## Anis had thos blocks. I don't think its needed. 
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

# find dependencies - I don't think all of them are requierd.  
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED) ## See ROS REP2000 for expected PCL version
find_package(visualization_msgs REQUIRED)
find_package(zion_msgs REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})



set(DEPENDENCIES
  rclcpp
  rclcpp_components
  rcutils
  zion_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

###############################################################################
# INCLUDES and LIBS

include_directories(
  include
  SYSTEM
  # ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


###############################################################################
# SOURCES


set(CLOUD_PROCESSOR_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/visibility_control.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/utilities.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/cloud_processor_component.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cloud_processor_component.cpp
)

set(BROADCASTER_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/visibility_control.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/zion_broadcaster_component.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/zion_broadcaster_component.cpp
)

set(STAIR_MODELING_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/visibility_control.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/plane.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/stair.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/utilities.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/cloud_processor_component.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/stair_modeling_component.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/zion_broadcaster_component.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/plane.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cloud_processor_component.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/stair_modeling_component.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/zion_broadcaster_component.cpp
)
###############################################################################
# Bin and Install

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(cloud_processor_component SHARED
    ${CLOUD_PROCESSOR_INC})
target_compile_definitions(cloud_processor_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(cloud_processor_component
    ${DEPENDENCIES})
rclcpp_components_register_nodes(cloud_processor_component "stair_modeling::CloudProcessor")
set(node_plugins "${node_plugins}stair_modeling::CloudProcessor;$<TARGET_FILE:cloud_processor_component>\n")

add_library(zion_broadcaster_component SHARED
    ${BROADCASTER_INC})
target_compile_definitions(zion_broadcaster_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(zion_broadcaster_component
    ${DEPENDENCIES})
rclcpp_components_register_nodes(zion_broadcaster_component "stair_modeling::ZionBroadcaster")
set(node_plugins "${node_plugins}stair_modeling::ZionBroadcaster;$<TARGET_FILE:zion_broadcaster_component>\n")

add_library(stair_modeling_component SHARED
    ${STAIR_MODELING_INC})
target_compile_definitions(stair_modeling_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(stair_modeling_component
    ${DEPENDENCIES})
rclcpp_components_register_nodes(stair_modeling_component "stair_modeling::StairModeling")
set(node_plugins "${node_plugins}stair_modeling::StairModeling;$<TARGET_FILE:stair_modeling_component>\n")

###############################################################################
# executables

add_executable(stair_modeling_ros
  src/stair_modeling_ros.cpp)
target_link_libraries(stair_modeling_ros
  cloud_processor_component
  zion_broadcaster_component
  stair_modeling_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(stair_modeling_ros
  ${DEPENDENCIES}
)

###############################################################################
# Install 

# Mark executables and/or libraries for installation

install(TARGETS
  cloud_processor_component
  zion_broadcaster_component
  stair_modeling_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  stair_modeling_ros
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY
  # config
  rviz2
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
