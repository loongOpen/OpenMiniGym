cmake_minimum_required(VERSION 2.8.3)
project(robot_sim)

set( CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  tf
  geometry_msgs
  pcl_conversions
  pcl_ros
)

generate_dynamic_reconfigure_options(
  ${PROJECT_SOURCE_DIR}/grasp/config/grasp.cfg
  ${PROJECT_SOURCE_DIR}/grasp/config/gpd.cfg
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  dynamic_reconfigure
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package( OpenCV   REQUIRED )

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/usr/include/eigen3"
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS} 
)


# 抓取部分 #
## geometric_method_grasp ##
include_directories(${PROJECT_SOURCE_DIR}/grasp/inc)

## GPD_method_grasp ##
add_executable(GPD_method_grasp ${PROJECT_SOURCE_DIR}/grasp/GPD_method/src/main.cpp)
add_dependencies(GPD_method_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GPD_method_grasp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

## GPD_method_grasp ##
add_executable(Geometric_method_grasp ${PROJECT_SOURCE_DIR}/grasp/geometric_method/src/main.cpp)
add_dependencies(Geometric_method_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Geometric_method_grasp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

