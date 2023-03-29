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
