cmake_minimum_required(VERSION 3.0.2)
project(waypoint_reconfigure VERSION 0.0.0 LANGUAGES CXX)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    tf
    geometry_msgs
    waypoint_manager_msgs
    dynamic_reconfigure
)

# generate_dynamic_reconfigure_options(
# cfg/DWAPlanner.cfg
# )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES goal_event_handler
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_reconfigure_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/waypoint_reconfigure_node.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(waypoint_reconfigure_node
    PROPERTIES
        OUTPUT_NAME waypoint_reconfigure_node
        VERSION ${PROJECT_VERSION}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(waypoint_reconfigure_node
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    yaml-cpp
)
target_compile_features(waypoint_reconfigure_node
    PRIVATE
        cxx_std_14
)