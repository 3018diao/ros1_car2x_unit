//
// Created by rosk on 30.08.19.
//

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <ublox_msgs/NavPVT.h>
#include <nav_msgs/Odometry.h>

#include "pdk_interface.hpp"

class VehicleInfoGenerator
{
public:
    VehicleInfoGenerator(ros::NodeHandle nh);
    void receiveGPS(const ublox_msgs::HnrPVT& gps_msg);
    void receiveFilterPose(const nav_msgs::Odometry& odom_msg);
    void transmitInfo();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Subscriber m_sub_localization;
    CVehicleDynamics m_pdkVehicleInfo;
};
