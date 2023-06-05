cmake_minimum_required(VERSION 3.0.2)
project(car2x_unit)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  cpm_interfaces
)


# find_package(Protobuf REQUIRED)
find_package(Flatbuffers REQUIRED PATHS "/home/huiyu/flatbuffers/CMake")

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
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

target_link_libraries(car2x_unit ${catkin_LIBRARIES})

