cmake_minimum_required(VERSION 2.8.3)
project(al5d_low_level_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -std=gnu++11 -Wall -Wextra)
set(CMAKE_MODULE_PATH ${CMAKE_HOME_DIRECTORY}/modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)
 find_package(ClangFormat)

 if(CLANG_FORMAT_FOUND)
        message("clang-format executable: ${CLANG_FORMAT_EXECUTABLE}")
        message("clang-format version: ${CLANG_FORMAT_VERSION}")
else()
        message("clang-format executable not found")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
    FILES 
    DoFConfiguration.msg
  )

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  EmergencyStopAl5d.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
  DIRECTORY action
  FILES
  Al5dConfiguration.action
  )

## Building the messages
generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs LIBRARIES
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Getting all the sources files
file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/LowLevelDriver/*.cpp")

## Creating and linking the executable
add_executable(${PROJECT_NAME} src/Main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} al5d_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(al5d_driver_test_program test/IntegrationTests.cpp ${SOURCE_FILES})
add_dependencies(al5d_driver_test_program al5d_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(al5d_driver_test_program ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test test/Main.cpp  test/AL5DLowLevelDriverTests.hpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}-test al5d_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})

#######################
## Clang formatting ##
#######################

FILE(GLOB CLANG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LowLevelDriver/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.ipp)
foreach(file ${CLANG_SOURCES})
   string(CONCAT ${CMAKE_SOURCE_DIR} file)
endforeach()

add_custom_target(
        ${PROJECT_NAME}_format
        COMMAND ${CLANG_FORMAT_EXECUTABLE}
        -i
        ${CLANG_SOURCES}
)

#############################
## Calculate test coverage ##
#############################

find_program(LCOV lcov)
IF(LCOV)
        include(CodeCoverage)
        set(COVERAGE_EXCLUDES '/usr/*' '${CMAKE_CURRENT_SOURCE_DIR}/test/*')
        SETUP_TARGET_FOR_COVERAGE(
        NAME ${PROJECT_NAME}_coverage
        EXECUTABLE ${PROJECT_NAME}-test -j ${PROCESSOR_COUNT}
        DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
        APPEND_COVERAGE_COMPILER_FLAGS()
ENDIF()