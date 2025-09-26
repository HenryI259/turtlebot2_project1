cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2_project1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
 roscpp 
 geometry_msgs
 nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot2_project1
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES})
add_dependencies(move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
