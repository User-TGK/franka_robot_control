cmake_minimum_required(VERSION 2.8.3)
project(high_level_interface)

add_compile_options(-std=c++11 -std=gnu++11 -Wall -Wextra -W -Wodr)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_HOME_DIRECTORY}/modules)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  roslib
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(ClangFormat)

if(CLANG_FORMAT_FOUND)
      message("clang-format executable: ${CLANG_FORMAT_EXECUTABLE}")
      message("clang-format version: ${CLANG_FORMAT_VERSION}")
else()
      message("clang-format executable not found")
endif()

add_message_files(
    FILES 
    DoFConfiguration.msg
)

add_action_files(
  DIRECTORY action
  FILES Configuration.action
)

add_service_files(
  FILES
  EmergencyStop.srv
  Position.srv
  Unlock.srv
  Gripper.srv
  UpdateManual.srv
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Getting all the sources files
file(GLOB_RECURSE SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/HighLevelInterface/*.cpp")

## Creating and linking the executable
add_executable(${PROJECT_NAME} src/Main.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} high_level_interface_generate_messages_cpp franka_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_integration_test test/ClientIntegrationTest ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_integration_test high_level_interface_generate_messages_cpp franka_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_integration_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_simulation_demo test/ClientSimulationDemo ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_simulation_demo high_level_interface_generate_messages_cpp franka_low_level_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_simulation_demo ${catkin_LIBRARIES})

#######################
## Clang formatting ##
#######################

FILE(GLOB CLANG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/HighLevelInterface/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.ipp)
foreach(file ${CLANG_SOURCES})
    string(CONCAT ${CMAKE_SOURCE_DIR} file)
endforeach()

add_custom_target(
        ${PROJECT_NAME}_format
        COMMAND ${CLANG_FORMAT_EXECUTABLE}
        -i
        ${CLANG_SOURCES}
)