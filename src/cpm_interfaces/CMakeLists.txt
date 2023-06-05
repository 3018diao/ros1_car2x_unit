cmake_minimum_required(VERSION 2.8.3)
project(cpm_interfaces)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)

add_message_files(
  FILES
  ${MESSAGE_FILES}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


