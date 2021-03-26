####################################################################################
# Copyright 2021 Robotic Systems Lab - ETH Zurich:                                 #
# Linghao Zhang, Jonas Junger, Lennart Nachtigall                                  #
#                                                                                  #
# Redistribution and use in source and binary forms, with or without               #
# modification, are permitted provided that the following conditions are met:      #
#                                                                                  #
# 1. Redistributions of source code must retain the above copyright notice,        #
#    this list of conditions and the following disclaimer.                         #
#                                                                                  #
# 2. Redistributions in binary form must reproduce the above copyright notice,     #
#    this list of conditions and the following disclaimer in the documentation     #
#    and/or other materials provided with the distribution.                        #
#                                                                                  #
# 3. Neither the name of the copyright holder nor the names of its contributors    #
#    may be used to endorse or promote products derived from this software without #
#    specific prior written permission.                                            #
#                                                                                  #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"      #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE        #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE   #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE     #
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL       #
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR       #
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,    #
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.             #
####################################################################################

cmake_minimum_required(VERSION 2.8.3)
project(maxon_epos_ethercat_sdk)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
  ethercat_sdk_master
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Maxon.cpp
  src/${PROJECT_NAME}/Configuration.cpp
  src/${PROJECT_NAME}/ConfigurationParser.cpp
  src/${PROJECT_NAME}/ConfigureParameters.cpp
  src/${PROJECT_NAME}/Diagnostics.cpp
  src/${PROJECT_NAME}/Reading.cpp
  src/${PROJECT_NAME}/Command.cpp
  src/${PROJECT_NAME}/Controlword.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/DriveState.cpp
  src/${PROJECT_NAME}/PdoTypeEnum.cpp
  src/${PROJECT_NAME}/ModeOfOperationEnum.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
