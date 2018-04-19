#
#  Copyright (C) 2017 IIT-ADVR
#  Author: Arturo Laurenzi
#  email: arturo.laurenzi@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

# CMake 3.5.1 required
cmake_minimum_required(VERSION 3.5.1)

# XBot RT Plugin
project(wholebody_keyboard_ctrl)

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find required packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs robot_state_publisher tf_conversions roscpp std_msgs sensor_msgs std_srvs geometry_msgs message_generation eigen_conversions)
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(SDL REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Key.msg
)

add_action_files(
  DIRECTORY action
  FILES LegMovement.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS robot_state_publisher 
                roscpp 
                actionlib_msgs
 DEPENDS ${Boost_LIBRARIES}
)

# message generation
# add_message_files(
# DIRECTORY msg
# FILES
# RobotState.msg
# )
# 
# add_service_files(
# DIRECTORY srv
# FILES
# Walk.srv
# CommandFootstep.srv
# )
# 
# generate_messages(
# DEPENDENCIES
# std_msgs
# geometry_msgs
# )

# include
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenSoT_INCLUDE_DIRS})

add_executable(keyboard_control src/keyboard_control.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES} ${Boost_LIBRARIES})

add_executable(demo_joy_control src/demo_joy_control.cpp)
target_link_libraries(demo_joy_control yaml-cpp ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES} ${Boost_LIBRARIES})

add_executable(centauro_wheeled_locomotion src/centauro_wheeled_locomotion.cpp)
target_link_libraries(centauro_wheeled_locomotion ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(wholebody_action_server src/wholebody_action_server.cpp)
target_link_libraries(wholebody_action_server ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES})

if(${SDL_FOUND})


include_directories(${SDL_INCLUDE_DIR})
add_library(keyboard src/keyboard.cpp)
add_dependencies(keyboard wholebody_keyboard_ctrl_gencpp)
target_link_libraries(keyboard ${SDL_LIBRARY})
add_executable(keyboard_to_joy src/keyboard_to_joy.cpp)
target_link_libraries(keyboard_to_joy ${catkin_LIBRARIES} keyboard)

ycm_library_install(keyboard 1 0 0)

INSTALL(TARGETS keyboard_to_joy
        EXPORT keyboard_to_joy
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
endif()



INSTALL(TARGETS keyboard_control
        EXPORT keyboard_control
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

INSTALL(TARGETS demo_joy_control
        EXPORT demo_joy_control
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
INSTALL(TARGETS centauro_wheeled_locomotion
        EXPORT centauro_wheeled_locomotion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

INSTALL(TARGETS wholebody_action_server
        EXPORT wholebody_action_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})