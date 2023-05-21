# Install script for directory: //home/huiyu/group1_service/src/car2x_unit

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/huiyu/group1_service/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car2x_unit/msg" TYPE FILE FILES
    "//home/huiyu/group1_service/src/car2x_unit/msg/Acceleration3dWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/AccelerationCartesian.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/AccelerationComponent.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/AccelerationMagnitude.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/AccelerationPolarWithZ.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ActionID.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Altitude.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/AngularSpeedConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/BasicVehicleContainerHighFrequency.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/BasicVehicleContainerLowFrequency.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CAMessage.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CPMessage.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CartesianAngle.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CartesianAngularVelocityComponent.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CartesianCoordinateWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CartesianPosition3d.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CartesianPosition3dWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CauseCode.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CauseCodeType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ChannelBusyRatio.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CircularShape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ClosedLanes.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CommandRequest.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CommandResponse.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CommandResponseData.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CorrelationColumn.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CpmPayload.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Curvature.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/CurvatureCalculationMode.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/DangerousGoodsBasic.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/DangerousGoodsExtended.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/DeltaReferencePosition.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/DriveDirection.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/EllipticalShape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/EnergyStorageType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/EulerAnglesWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/EventHistory.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/EventPoint.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/FacilityLayerMessage.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/FacilityLayerReception.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/FacilityLayerTransmission.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/GossipMessage.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/HardShoulderStatus.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Heading.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/IntersectionReferenceID.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ItineraryPath.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ItsPduHeader.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LifecycleAction.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LightBarSirenInUse.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LinkLayerPriority.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LinkLayerReception.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LinkLayerTransmission.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LongitudinalAcceleration.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LongitudinalLanePosition.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LowerTriangularPositiveSemidefiniteMatrix.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/LowerTriangularPositiveSemidefiniteMatrixColumns.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ManagmentContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MapPosition.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MapReference.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MatrixIncludedComponents.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MessageRateHz.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MessageRateRange.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/MessageSegmentationInfo.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ObjectClass.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ObjectClassWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ObjectDimension.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/OriginatingRSUContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/OriginatingStationsContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/OriginatingVehicleContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/OtherSubClass.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PerceivedObject.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PerceivedObjectContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PerceptionRegion.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PerceptionRegionContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PolygonalShape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PositionConfidenceEllipse.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PositionOfOccupants.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PositioningSolutionType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/PtActivationType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RadialShape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RadialShapeDetails.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RadialShapes.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RadioConfiguration.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RectangularShape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ReferenceDenms.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/ReferencePosition.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RelevanceDistance.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RelevanceTrafficDirection.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RequestResponseIndication.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RestrictedTypes.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RoadSegmentReferenceID.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/RoadType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/SPATState.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/SensorInformation.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/SensorInformationContainer.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/SensorType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Shape.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/SignalStatus.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Speed.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/StationType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/StationaryObject.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/StationarySince.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/StationaryVehicle.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/TrafficLightState.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/TrafficParticipantType.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/TrafficRule.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VehicleIdentification.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VehicleLength.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VehicleLengthConfidenceIndication.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VehicleWidth.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Velocity3dWithConfidence.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VelocityCartesian.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VelocityComponent.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VelocityPolarWithZ.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruClusterInformation.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruClusterProfiles.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruProfileAndSubprofile.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruSubProfileAnimal.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruSubProfileBicyclist.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruSubProfileMotorcyclist.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/VruSubProfilePedestrian.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/Wgs84Angle.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/YawRate.msg"
    "//home/huiyu/group1_service/src/car2x_unit/msg/YawRateConfidence.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car2x_unit/cmake" TYPE FILE FILES "/home/huiyu/group1_service/build/car2x_unit/catkin_generated/installspace/car2x_unit-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/huiyu/group1_service/devel/include/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/huiyu/group1_service/devel/share/roseus/ros/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/huiyu/group1_service/devel/share/common-lisp/ros/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/huiyu/group1_service/devel/share/gennodejs/ros/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/huiyu/group1_service/devel/lib/python3/dist-packages/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/huiyu/group1_service/devel/lib/python3/dist-packages/car2x_unit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/huiyu/group1_service/build/car2x_unit/catkin_generated/installspace/car2x_unit.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car2x_unit/cmake" TYPE FILE FILES "/home/huiyu/group1_service/build/car2x_unit/catkin_generated/installspace/car2x_unit-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car2x_unit/cmake" TYPE FILE FILES
    "/home/huiyu/group1_service/build/car2x_unit/catkin_generated/installspace/car2x_unitConfig.cmake"
    "/home/huiyu/group1_service/build/car2x_unit/catkin_generated/installspace/car2x_unitConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car2x_unit" TYPE FILE FILES "//home/huiyu/group1_service/src/car2x_unit/package.xml")
endif()

