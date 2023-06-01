cmake_minimum_required(VERSION 3.0.2)
project(car2x_unit)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
)


find_package(Protobuf REQUIRED)
find_package(Flatbuffers REQUIRED PATHS "/home/huiyu/flatbuffers/CMake")


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
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/car2x_unit
  /home/huiyu/flatbuffers/include
)

add_executable(car2x_unit
  src/car2x_unit.cpp
)

catkin_install_python(
  PROGRAMS
    scripts/group4_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS car2x_unit
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(car2x_unit ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

