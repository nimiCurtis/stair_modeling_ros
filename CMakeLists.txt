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

# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
find_package(PCL REQUIRED) ## See ROS REP2000 for expected PCL version

# find_package(rcl_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(eigen REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
#Custom
find_package(zion_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC_FILES
  src/plane.cpp
  src/stair_modeling.cpp
  src/stair.cpp
  src/utilities.cpp
  src/zion_tf2_broadcaster.cpp
  src/cloud_processor.cpp
)

set(dependencies
  rclcpp
  rcutils
  zion_msgs
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(stair_modeling_ros_node
  src/stair_modeling_ros_node.cpp
)

target_link_libraries(stair_modeling_ros_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

########### zion_tf2_broadcaster #####
# add_executable(zion_tf2_broadcaster src/zion_tf2_broadcaster.cpp)

# ament_target_dependencies(zion_tf2_broadcaster 
#   ${dependencies} 
#   )

#############
## Install ## 
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
  ${PROJECT_NAME}
  stair_modeling_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
