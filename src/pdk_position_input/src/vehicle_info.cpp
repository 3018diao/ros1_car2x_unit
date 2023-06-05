//
// Created by rosk on 30.08.19.
//

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include <ublox_msgs/HnrPVT.h>
#include <nav_msgs/Odometry.h>
#include "vehicle_info.hpp"




VehicleInfoGenerator::VehicleInfoGenerator(ros::NodeHandle nh): m_nh(nh){

    bool use_pdk_localization;
    
    if(!m_nh.hasParam("use_pdk_localization")){
        ROS_WARN("No indication to pdk localization input, waiting for Ublox");
    }

    m_nh.getParam("use_pdk_localization", use_pdk_localization);
    std::cout << "param: " << use_pdk_localization << std::endl;
    if(use_pdk_localization){
        m_sub_localization = nh.subscribe("/odometry/filtered_map", 1000, &VehicleInfoGenerator::receiveFilterPose, this);
    }else{
        m_sub = nh.subscribe("/gnss/hnrpvt", 1000, &VehicleInfoGenerator::receiveGPS, this);
    }
    PDK::CInterface::Init("/opt/pdk/etc/pdk_config.json");
    //transmitInfo();
}


void VehicleInfoGenerator::receiveGPS(const ublox_msgs::HnrPVT& gps_msg) {
    const auto tNow = std::chrono::system_clock::now();

    m_pdkVehicleInfo.TimeStamp = CTimeStamp(std::chrono::duration_cast<std::chrono::seconds>(tNow.time_since_epoch()).count(), std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch()).count());
    m_pdkVehicleInfo.LongVel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LongVel.value = gps_msg.gSpeed / 1000.0; //from mm to m/s
    m_pdkVehicleInfo.YawRate.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.YawRate.value = 0; //from mm to degrees 
    m_pdkVehicleInfo.LongAccel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LongAccel.value = 0.0; //from mm to m/s
    m_pdkVehicleInfo.LatAccel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LatAccel.value = 0.0; //from mm to m/s

    bool res = PDK::CInterface::PublishVehicleDynamics(m_pdkVehicleInfo);
    if(!res){
        std::cout << "Error while publishing vehicle dynamics " << std::endl;
    }
}

void VehicleInfoGenerator::receiveFilterPose(const nav_msgs::Odometry& odom_msg) {
    const auto tNow = std::chrono::system_clock::now();
    std::cout << "Received sometihng " << std::endl;
    m_pdkVehicleInfo.TimeStamp = CTimeStamp(std::chrono::duration_cast<std::chrono::seconds>(tNow.time_since_epoch()).count(), std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch()).count());
    m_pdkVehicleInfo.LongVel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LongVel.value = odom_msg.twist.twist.linear.x ;
    m_pdkVehicleInfo.YawRate.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.YawRate.value = 0; //from mm to degrees 
    m_pdkVehicleInfo.LongAccel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LongAccel.value = 0.0; //from mm to m/s
    m_pdkVehicleInfo.LatAccel.state = CVehicleDynamicsSignal::eSignalState::VALID;
    m_pdkVehicleInfo.LatAccel.value = 0.0; //from mm to m/s

    bool res = PDK::CInterface::PublishVehicleDynamics(m_pdkVehicleInfo);
    if(!res){
        std::cout << "Error while publishing vehicle dynamics " << std::endl;
    }
}

void VehicleInfoGenerator::transmitInfo() {
    ros::Rate r(20);
    bool res;
    while(ros::ok()){
        res = PDK::CInterface::PublishVehicleDynamics(m_pdkVehicleInfo);

        if(!res){
            std::cout << "Error while publishing vehicle dynamics " << std::endl;
        }
        
        r.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pdk_input");

    ros::NodeHandle nh("~");
    VehicleInfoGenerator vehicleInfoGenerator(nh);

    ros::spin();
}
