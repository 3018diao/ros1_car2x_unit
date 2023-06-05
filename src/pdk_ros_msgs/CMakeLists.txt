cmake_minimum_required(VERSION 2.8.3)
project(pdk_ros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  pdk_version.msg
  pdk_SignalSource.msg
  pdk_time.msg
  pdk_SignalStatus.msg
  pdk_SensorType.msg
  pdk_SensorMsgHeader.msg
  pdk_MsgHeader.msg  
  pdk_RadarObjectList.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
  DEPENDS message_generation
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


catkin_install_python(
  PROGRAMS
    scripts/pdk_tracking_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

