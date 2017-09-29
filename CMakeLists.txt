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
project(wholebody-keyboard-ctrl)

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
find_package(catkin REQUIRED COMPONENTS robot_state_publisher tf_conversions roscpp sensor_msgs std_srvs geometry_msgs message_generation eigen_conversions)
find_package(OpenSoT REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)


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

INSTALL(TARGETS keyboard_control
        EXPORT keyboard_control
        RUNTIME DESTINATION bin)