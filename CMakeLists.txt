cmake_minimum_required(VERSION 2.8.3)
project(rosilo_denso_communication)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Werror=return-type)
add_definitions(-D_USE_LINUX_API)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    rosilo_datalogger
    rosilo_clock
    rosilo_conversions
    rosilo_robot_driver
    rosilo_robot_driver_denso
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rosilo_denso_communication
    CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs rosilo_clock rosilo_conversions rosilo_robot_driver rosilo_robot_driver_denso
    # DEPENDS dqrobotics
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(rosilo_denso_communication
    src/rosilo_denso_communication_interface.cpp
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(rosilo_denso_communication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(rosilo_denso_communication_node
    src/rosilo_denso_communication_node.cpp
    )

add_executable(rosilo_denso_communication_dummy_node
    src/rosilo_denso_communication_dummy_node.cpp
    )

add_executable(rosilo_denso_communication_vrep_node
    src/rosilo_denso_communication_vrep_node.cpp
    )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rosilo_denso_communication_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rosilo_denso_communication_vrep_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rosilo_denso_communication_dummy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosilo_denso_communication_node
    dqrobotics
    rosilo_denso_communication
    ${catkin_LIBRARIES}
    )

target_link_libraries(rosilo_denso_communication_dummy_node
    dqrobotics
    ${catkin_LIBRARIES}
    )

target_link_libraries(rosilo_denso_communication_vrep_node
    dqrobotics
    dqrobotics-interface-vrep
    ${catkin_LIBRARIES}
    )

