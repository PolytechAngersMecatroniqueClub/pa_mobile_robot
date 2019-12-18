cmake_minimum_required(VERSION 2.8.3)
project(pa_mobile_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pa_mobile_robot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(odom2path_node src/tools/odom2path_node.cpp
)

## Declare a C++ executable
add_executable(pa_slam_node pa_slam_node.cpp
    src/slam/PaSlam_Costmap.cpp
    src/slam/PaSlam_Map.cpp
    src/slam/PaSlam.cpp
    )

## Declare a C++ executable
add_executable(pa_exploration_node pa_exploration_node.cpp
    src/exploration/PaPoint2Di.cpp
    src/exploration/PaNode.cpp
    src/exploration/PaExplorationMap.cpp
    src/exploration/PaAStar.cpp
    )

## Declare a C++ executable
add_executable(pa_exploration_slam_node pa_exploration_slam_node.cpp
    src/exploration/PaPoint2Di.cpp
    src/exploration/PaNode.cpp
    src/exploration/PaExplorationMap.cpp
    src/exploration/PaAStar.cpp
    src/slam/PaSlam_Costmap.cpp
    src/slam/PaSlam_Map.cpp
    src/slam/PaSlam.cpp
    )

target_link_libraries(pa_exploration_slam_node
    ${catkin_LIBRARIES}
)

target_link_libraries(pa_slam_node
    ${catkin_LIBRARIES}
)

target_link_libraries(pa_exploration_node
    ${catkin_LIBRARIES}
)

target_link_libraries(odom2path_node
    ${catkin_LIBRARIES}
)

