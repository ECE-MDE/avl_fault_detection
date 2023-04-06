#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_fault_detection ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 3.0.2)

# Package name
project(avl_fault_detection)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-psabi)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: no build type specified, choosing Release")
  set(CMAKE_BUILD_TYPE Release)
endif()


#===============================================================================
#                              FIND DEPENDENCIES
#===============================================================================

# Find other Catkin packages required for build
find_package(catkin REQUIRED COMPONENTS
    avl_asio
    avl_msgs
    avl_control
    avl_core
    avl_guidance
    avl_navigation
    roscpp
    std_srvs
    std_msgs
    rospy
)

# Find the Eigen3 package required for build
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

#===============================================================================
#                           MESSAGES/SERVICES/ACTIONS
#===============================================================================

#===============================================================================
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
        avl_asio
        avl_msgs
        avl_control 
        avl_core 
        avl_guidance 
        avl_navigation 
        roscpp
        std_srvs
    DEPENDS
    CFG_EXTRAS
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/listener.py
  src/data_collection_launcher.py
  src/data_collector.py
  src/simple_fault_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Nodes ------------------------------------------------------------------------
# Attitude control node for Zodiac
add_executable(ekf_fault_detection_node src/ekf_fault_detection_node.cpp)
add_dependencies(ekf_fault_detection_node ${EXPORTED_TARGETS})
target_link_libraries(ekf_fault_detection_node node ${catkin_LIBRARIES})
