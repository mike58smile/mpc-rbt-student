cmake_minimum_required(VERSION 3.8)
project(mpc_rbt_student)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# # Use C++ 17
# SET(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# # Setup Debug and Release flags
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
# set(CMAKE_BUILD_TYPE "Debug")

# add_compile_options(-Wno-unused-parameter -Wno-unused-variable)

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)

include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  nav_msgs
  mpc_rbt_simulator
)
add_library(${PROJECT_NAME} SHARED
  src/KeyboardControl.cpp
  src/Localization.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(KeyboardControl src/keyboard_control_node.cpp)
target_link_libraries(KeyboardControl ${PROJECT_NAME})
install(TARGETS KeyboardControl DESTINATION lib/${PROJECT_NAME})  

# Add LocalizationNode
add_executable(LocalizationNode src/localization_node.cpp)
target_link_libraries(LocalizationNode ${PROJECT_NAME})
install(TARGETS LocalizationNode DESTINATION lib/${PROJECT_NAME})

#Define include directories for the compiler
#include_directories(PUBLIC
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<INSTALL_INTERFACE:include>)

#add_executable(TestNode src/TestNode.cpp)

#target_include_directories(TestNode PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(TestNode rclcpp std_msgs sensor_msgs geometry_msgs)

#install(TARGETS TestNode DESTINATION lib/${PROJECT_NAME})
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
# Install rviz configuration files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries({PROJECT_NAME})
ament_package()
