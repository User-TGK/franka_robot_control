cmake_minimum_required(VERSION 2.8.3)
project(franka_low_level_simulation_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -std=gnu++11 -Wall -Wextra -W -Wodr)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.3.0 REQUIRED )

IF (Franka)
    message ( "LibFranka found" )
ELSE ()
    message ( "LibFranka not found" )
ENDIF()

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
  EmergencyStopFranka.srv
  UnlockFranka.srv
  GripperFranka.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
  DIRECTORY action
  FILES
  FrankaConfiguration.action
  )

## Building the messages
generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs message_runtime actionlib_msgs
  LIBRARIES franka_state_controller franka_control_services 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/LowLevelDriver/*.cpp")

add_executable(${PROJECT_NAME} src/Main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} franka_low_level_simulation_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Franka_LIBRARIES} ${catkin_LIBRARIES})

#############
## Testing ##
#############

add_executable(franka_simulation_driver_test_program test/FrankaConfigurationTest.cpp ${SOURCE_FILES})
add_dependencies(franka_simulation_driver_test_program franka_low_level_simulation_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_simulation_driver_test_program ${Franka_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test test/Main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}-test franka_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}-test ${Franka_LIBRARIES} ${catkin_LIBRARIES})

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
