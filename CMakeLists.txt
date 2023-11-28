cmake_minimum_required(VERSION 2.8.3)
project(visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  tf
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7  REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(local_visualization src/local_visualization.cpp)
target_link_libraries(local_visualization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(local_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(global_visualization src/global_visualization.cpp)
target_link_libraries(global_visualization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(global_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})