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

cmake_minimum_required(VERSION 3.16)
project(maxon_epos_ethercat_sdk)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 17)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
    ament_cmake
    message_logger
    ethercat_sdk_master
    yaml-cpp
    )

find_package(ament_cmake REQUIRED)
find_package(message_logger REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(yaml-cpp REQUIRED)


###########
## Build ##
###########


FILE(GLOB CPPSources src/${PROJECT_NAME}/*.cpp)
add_library(${PROJECT_NAME}
    ${CPPSources}
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
    ${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})


#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

ament_package()