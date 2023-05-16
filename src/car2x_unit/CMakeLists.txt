cmake_minimum_required(VERSION 3.0.2)
project(car2x_unit)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Protobuf REQUIRED)


########
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/car2x_unit
)

add_executable(car2x_unit src/car2x_unit.cpp src/cpm_interface.pb.cc src/etsi_its_cdd.pb.cc src/itspduheader_interface.pb.cc src/cam_interface.pb.cc src/denm_interface.pb.cc src/etsi_msg_interface.pb.cc)


# install(TARGETS car2x_unit
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(car2x_unit ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

