
cmake_minimum_required(VERSION 3.0)
project(maxon_sdo_only_example)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)


set(PACKAGE_DEPENDENCIES
 ethercat_device_configurator
 maxon_epos_ethercat_sdk
 message_logger
 roscpp
 std_msgs
 message_generation
)

find_package(catkin REQUIRED
        COMPONENTS
        ${PACKAGE_DEPENDENCIES}
        )

add_service_files(DIRECTORY msg FILES 
  trigger_velocity_change.srv
)
add_message_files(DIRECTORY msg FILES 
  simple_motor_reading.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
    std_msgs
)

include_directories(
  include
  ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/BasicMaxonDriveManager.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)


add_executable(
  basic_maxon_drive_manager_node
  src/basic_maxon_drive_manager_node.cpp
)
add_dependencies(
  basic_maxon_drive_manager_node
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  basic_maxon_drive_manager_node
  ${PROJECT_NAME}
  ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  -pthread
  stdc++fs
)


install(TARGETS ${PROJECT_NAME} basic_maxon_drive_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



