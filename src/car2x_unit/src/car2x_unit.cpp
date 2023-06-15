#include <ros/ros.h>
#include "std_msgs/String.h"
// #include "cpm_interface.pb.h"
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <ros/message_operations.h>
#include <sstream>
#include "cpm_interfaces/PerceivedObjectContainer.h"
#include "etsi_msg_interface_generated.h"
// #include "gossip_msg_generated.h"
// #include "cpm_interface_generated.h"
#include "all_interface_generated.h"
#include "cpm_interfaces/GossipMessage.h"
#include <chrono>

using namespace Gos;
using boost::asio::ip::udp;
using namespace std;
// using namespace Gossip;
// using namespace cpm_interfaces;
using namespace Gos;

/**
 * @brief Convert a flatbuffers vector to a std::vector.
 */
template <typename T>
std::vector<T> ConvertFlatbuffersVector(const flatbuffers::Vector<T> *fbVector)
{
    std::vector<T> result;
    result.reserve(fbVector->size());
    for (const auto &value : *fbVector)
    {
        result.push_back(static_cast<T>(value));
    }
    return result;
}

/**
 * @brief Class for the car2x unit.
 *
 * This class is responsible for the communication with the car2x unit.
 * It sends and receives messages to/from the cohda-box.
 *
 */
class Car2xUnit
{
public:
    Car2xUnit(const std::string &ip_address, unsigned short local_port, unsigned short remote_port)
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<cpm_interfaces::GossipMessage>("cpm", 10);
        subscriber_ = nh.subscribe("collective_perception", 1000, &Car2xUnit::collectivePerceptionCallback, this);

        socket_ = make_unique<udp::socket>(io_context_, udp::endpoint(udp::v4(), local_port));

        remote_endpoint_ = udp::endpoint(boost::asio::ip::address::from_string(ip_address), remote_port);

        start_receive();
        io_thread_ = make_unique<thread>([this]()
                                         { io_context_.run(); });
    }

    ~Car2xUnit()
    {
        io_context_.stop();
        if (io_thread_ && io_thread_->joinable())
            io_thread_->join();
    }

private:
    void start_receive()
    {
        socket_->async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&Car2xUnit::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    /**
     * @brief Callback function for the collective perception topic.
     *
     * @param msg
     */
    void collectivePerceptionCallback(const cpm_interfaces::PerceivedObjectContainer::ConstPtr &rosPerceivedObjectContainer)
    {
        flatbuffers::FlatBufferBuilder builder;

        vector<flatbuffers::Offset<Gos::PerceivedObject>> perceived_objects;

        auto perceivedObjects = rosPerceivedObjectContainer->perceivedObjects;

        for (const auto &perceivedObject : perceivedObjects)
        {

            auto x_cord = Gos::CreateCartesianCoordinateWithConfidence(builder, perceivedObject.position.x_cord.value, perceivedObject.position.x_cord.confidence);

            auto y_cord = Gos::CreateCartesianCoordinateWithConfidence(builder, perceivedObject.position.y_cord.value, perceivedObject.position.y_cord.confidence);

            auto z_cord = Gos::CreateCartesianCoordinateWithConfidence(builder, perceivedObject.position.z_cord.value, perceivedObject.position.z_cord.confidence);

            auto position = Gos::CreateCartesianPosition3dWithConfidence(builder, x_cord, y_cord, z_cord);

            //

            auto x_velocity = Gos::CreateVelocityComponent(builder, perceivedObject.velocity.cartesianVelocity.xVelocity.vel_comp_value, perceivedObject.velocity.cartesianVelocity.xVelocity.speed_confidence);

            auto y_velocity = Gos::CreateVelocityComponent(builder, perceivedObject.velocity.cartesianVelocity.yVelocity.vel_comp_value, perceivedObject.velocity.cartesianVelocity.yVelocity.speed_confidence);

            auto z_velocity = Gos::CreateVelocityComponent(builder, perceivedObject.velocity.cartesianVelocity.zVelocity.vel_comp_value, perceivedObject.velocity.cartesianVelocity.zVelocity.speed_confidence);

            auto cartesian_velocity = Gos::CreateVelocityCartesian(builder, x_velocity, y_velocity, z_velocity);

            auto velocity_magnitude = Gos::CreateSpeed(builder, perceivedObject.velocity.polarVelocity.velocityMagnitude.value, perceivedObject.velocity.polarVelocity.velocityMagnitude.confidence);

            auto velocity_direction = Gos::CreateCartesianAngle(builder, perceivedObject.velocity.polarVelocity.velocityDirection.cartesian_value, perceivedObject.velocity.polarVelocity.velocityDirection.cartesian_angle_confidence);

            auto polar_velocity_z_velocity = Gos::CreateVelocityComponent(builder, perceivedObject.velocity.polarVelocity.zVelocity.vel_comp_value, perceivedObject.velocity.polarVelocity.zVelocity.speed_confidence);

            //

            auto polar_velocity = Gos::CreateVelocityPolarWithZ(builder, velocity_magnitude, velocity_direction, polar_velocity_z_velocity);

            auto velocity = Gos::CreateVelocity3dWithConfidence(builder, polar_velocity, cartesian_velocity);

            // acceleration

            auto acceleration_magnitude = Gos::CreateAccelerationMagnitude(builder, perceivedObject.acceleration.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue, perceivedObject.acceleration.polarAcceleration.accelerationMagnitude.accelerationConfidence);

            auto acceleration_direction = Gos::CreateCartesianAngle(builder, perceivedObject.acceleration.polarAcceleration.accelerationDirection.cartesian_value, perceivedObject.acceleration.polarAcceleration.accelerationDirection.cartesian_angle_confidence);

            // auto z_acceleration = Gos::CreateAccel(builder);

            Gos::AccelerationComponent polar_z_acceleration(perceivedObject.acceleration.polarAcceleration.zAcceleration.value, perceivedObject.acceleration.polarAcceleration.zAcceleration.confidence);

            auto polar_acceleration = Gos::CreateAccelerationPolarWithZ(builder, acceleration_magnitude, acceleration_direction, &polar_z_acceleration);

            Gos::AccelerationComponent x_acceleration(perceivedObject.acceleration.cartesianAcceleration.xAcceleration.value, perceivedObject.acceleration.cartesianAcceleration.xAcceleration.confidence);

            Gos::AccelerationComponent y_acceleration(perceivedObject.acceleration.cartesianAcceleration.yAcceleration.value, perceivedObject.acceleration.cartesianAcceleration.yAcceleration.confidence);

            Gos::AccelerationComponent z_acceleration(perceivedObject.acceleration.cartesianAcceleration.zAcceleration.value, perceivedObject.acceleration.cartesianAcceleration.zAcceleration.confidence);

            auto cartesian_acceleration = Gos::CreateAccelerationCartesian(builder, &x_acceleration, &y_acceleration, &z_acceleration);

            auto acceleration = Gos::CreateAcceleration3dWithConfidence(builder, polar_acceleration, cartesian_acceleration);

            // angles

            auto z_angle = Gos::CreateCartesianAngle(builder, perceivedObject.angles.zAngle.cartesian_value, perceivedObject.angles.zAngle.cartesian_angle_confidence);

            auto y_angle = Gos::CreateCartesianAngle(builder, perceivedObject.angles.yAngle.cartesian_value, perceivedObject.angles.yAngle.cartesian_angle_confidence);

            auto x_angle = Gos::CreateCartesianAngle(builder, perceivedObject.angles.xAngle.cartesian_value, perceivedObject.angles.xAngle.cartesian_angle_confidence);

            auto angles = Gos::CreateEulerAnglesWithConfidence(builder, z_angle, y_angle, x_angle);

            // z_angular_velocity

            auto z_angular_velocity_confidence = static_cast<Gos::AngularSpeedConfidence>(perceivedObject.zAngularVelocity.confidence.confi);

            auto z_angular_velocity = Gos::CreateCartesianAngularVelocityComponent(builder, perceivedObject.zAngularVelocity.value, z_angular_velocity_confidence);

            // lower_triangular_correlation_matrices

            vector<flatbuffers::Offset<Gos::LowerTriangularPositiveSemidefiniteMatrix>> lower_triangular_correlation_matrices;

            auto lowerTriangularCorrelationMatrices = perceivedObject.lowerTriangularCorrelationMatrices;

            for (const auto &lowerTriangularCorrelationMatrix : lowerTriangularCorrelationMatrices)
            {
                auto components_included_in_the_matrix = Gos::CreateMatrixIncludedComponents(builder,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.xPosition,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.yPosition,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.zPosition,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.xVelocityOrVelocityMagnitude,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.yVelocityOrVelocityDirection,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.zSpeed,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.xAccelOrAccelMagnitude,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.yAccelOrAccelDirection,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.zAcceleration,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.zAngle,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.yAngle,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.xAngle,
                                                                                             lowerTriangularCorrelationMatrix.componentsIncludedIntheMatrix.zAngularVelocity);

                auto matrix = Gos::CreateLowerTriangularPositiveSemidefiniteMatrixColumns(builder);

                auto lower_triangular_correlation_matrix = Gos::CreateLowerTriangularPositiveSemidefiniteMatrix(builder, components_included_in_the_matrix, matrix);
                lower_triangular_correlation_matrices.push_back(lower_triangular_correlation_matrix);
            }

            auto lower_triangular_correlation_matrices_vector = builder.CreateVector(lower_triangular_correlation_matrices);

            // object_dimension

            auto object_dimension_z = Gos::CreateObjectDimension(builder, perceivedObject.objectDimensionZ.value, perceivedObject.objectDimensionZ.confidence);

            auto object_dimension_y = Gos::CreateObjectDimension(builder, perceivedObject.objectDimensionY.value, perceivedObject.objectDimensionY.confidence);

            auto object_dimension_x = Gos::CreateObjectDimension(builder, perceivedObject.objectDimensionX.value, perceivedObject.objectDimensionX.confidence);

            // sensor_id_list

            vector<uint32_t> sensor_ids = {1, 2, 3};

            auto sensor_id_list = builder.CreateVector(perceivedObject.sensorIdList);

            //
            vector<flatbuffers::Offset<Gos::ObjectClassWithConfidence>> rosClassification;

            for (const auto &objectClassWithConfidence : perceivedObject.classification)
            {
                auto object_class = Gos::CreateObjectClass(builder);
                auto object_class_with_confidence = Gos::CreateObjectClassWithConfidence(builder);
                rosClassification.push_back(object_class_with_confidence);
            }

            auto classification = builder.CreateVector(rosClassification);

            // map_position

            auto road_segment = Gos::CreateRoadSegmentReferenceID(builder, perceivedObject.mapPosition.mapReference.roadsegment.region, perceivedObject.mapPosition.mapReference.roadsegment.id);

            auto intersection = Gos::CreateIntersectionReferenceID(builder, perceivedObject.mapPosition.mapReference.intersection.region, perceivedObject.mapPosition.mapReference.intersection.id);

            auto map_reference = Gos::CreateMapReference(builder, road_segment, intersection);

            auto longitudinal_lane_position = Gos::CreateLongitudinalLanePosition(builder, perceivedObject.mapPosition.longitudinalLanePosition.longitudinalLanePositionValue, perceivedObject.mapPosition.longitudinalLanePosition.longitudinalLanePositionConfidence);

            auto map_position = Gos::CreateMapPosition(builder, map_reference, perceivedObject.mapPosition.laneId, perceivedObject.mapPosition.connectionId, longitudinal_lane_position);

            // todo: add the values to the dimensions
            auto perceived_object = Gos::CreatePerceivedObject(builder,
                                                               perceivedObject.objectID,
                                                               perceivedObject.measurementDeltaTime,
                                                               position,
                                                               velocity,
                                                               acceleration,
                                                               angles,
                                                               z_angular_velocity,
                                                               lower_triangular_correlation_matrices_vector,
                                                               object_dimension_z,
                                                               object_dimension_y,
                                                               object_dimension_x,
                                                               perceivedObject.objectAge,
                                                               perceivedObject.objectPerceptionQuality,
                                                               sensor_id_list,
                                                               classification,
                                                               map_position);

            perceived_objects.push_back(perceived_object);
        }

        auto perceived_object_vector = builder.CreateVector(perceived_objects);

        auto perceived_object_container = CreatePerceivedObjectContainer(builder, rosPerceivedObjectContainer->numberOfPerceivedObjects, perceived_object_vector);

        auto perception_region_container = Gos::CreatePerceptionRegionContainer(builder);

        auto sensor_information_container = Gos::CreateSensorInformationContainer(builder);

        auto originating_stations_container = Gos::CreateOriginatingStationsContainer(builder);

        auto cpm_payload = Gos::CreateCpmPayload(builder, originating_stations_container, sensor_information_container, perception_region_container, perceived_object_container);

        auto altitude = Gos::CreateAltitude(builder);

        auto position_confidence = Gos::CreatePositionConfidenceEllipse(builder);

        auto referencePosition = Gos::CreateReferencePosition(builder, 0, 0, position_confidence, altitude);

        auto management_container = CreateManagementContainer(builder, 0, referencePosition);

        auto now = std::chrono::system_clock::now();

        auto duration = now.time_since_epoch();

        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

        int timestampMs = static_cast<int>(milliseconds);

        auto generation_delta_time = static_cast<int>(milliseconds);

        auto cpm_its_pdu_header = Gos::CreateItsPduHeader(builder, 0, 0, 0);

        auto cpm_msg = Gos::CreateCPMessage(builder, cpm_its_pdu_header, generation_delta_time, management_container, cpm_payload);

        // auto cam_its_pdu_header = Gos::CreateItsPduHeader(builder, 0, 0, 0);

        // auto positionConfidenceEllipse = Gos::CreatePositionConfidenceEllipse(builder, 0, 0, 0);

        // Gos::StationType_STN_TYPE_UNKNOWN

        // auto station_type = Gos::StationType_STN_TYPE_UNKNOWN;

        // auto cam_msg = Gos::CreateCAMessage(builder, cam_its_pdu_header, 0, Gos::StationType_STN_TYPE_UNKNOWN, referencePosition, 0);

        auto facility_layer_message = Gos::CreateFacilityLayerMessage(builder, 0, cpm_msg);

        auto facility_layer_reception = CreateFacilityLayerReception(builder, facility_layer_message);

        auto gossip_message = CreateGossipMessage(builder, GossipType_FacilityLayerReception, facility_layer_reception.Union());

        builder.Finish(gossip_message);

        uint8_t *buf = builder.GetBufferPointer();
        int size = builder.GetSize();

        boost::shared_ptr<std::vector<uint8_t>> message(new std::vector<uint8_t>(buf, buf + size));

        socket_->async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                               boost::bind(&Car2xUnit::handle_send, this, message,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));

        // ostringstream oss;
        // ros::message_operations::Printer<cpm_interfaces::PerceivedObjectContainer>::stream(oss, "", *msg);
        ROS_INFO("****************");
        // ROS_INFO("%s", oss.str().c_str());
    }

    void handle_send(boost::shared_ptr<vector<uint8_t>> /*message*/,
                     const boost::system::error_code &error,
                     size_t bytes_transferred)
    {
        if (!error)
        {
            // cout << "Message sent! Bytes transferred: " << bytes_transferred << endl;
            ROS_INFO("Message sent! Bytes transferred: %d", bytes_transferred);
        }
        else
        {
            // cout << "Failed to send message: " << error.message() << endl;
            ROS_ERROR("Failed to send message: %s", error.message().c_str());
        }
    }

    /**
     * @brief Handle the received message.
     *
     * @param error
     * @param bytes_transferred
     */
    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
    {
        if (!error)
        {
            cpm_interfaces::GossipMessage rosGossipMessage;

            rosGossipMessage.is_linklayer_rx = false;
            rosGossipMessage.is_cbr = false;
            rosGossipMessage.is_facilitylayer_rx = false;

            auto gossipMessage = GetGossipMessage(recv_buffer_.data());

            // Check the type of the `gossip` field and handle each type.

            switch (gossipMessage->gossip_type())
            {
            case GossipType_ChannelBusyRatio:
            {
                rosGossipMessage.is_cbr = true;
                auto busy_ratio = gossipMessage->gossip_as_ChannelBusyRatio();
                rosGossipMessage.cbr.busy = busy_ratio->busy();
                rosGossipMessage.cbr.total = busy_ratio->total();
                break;
            }
            case GossipType_LinkLayerReception:
            {
                rosGossipMessage.is_linklayer_rx = true;
                auto ll_reception = gossipMessage->gossip_as_LinkLayerReception();
                rosGossipMessage.linklayer_rx.channel = ll_reception->channel();
                rosGossipMessage.linklayer_rx.destination = ConvertFlatbuffersVector(ll_reception->destination());
                rosGossipMessage.linklayer_rx.payload = ConvertFlatbuffersVector(ll_reception->payload());
                rosGossipMessage.linklayer_rx.power_cbm = ll_reception->power_cbm();
                rosGossipMessage.linklayer_rx.source = ConvertFlatbuffersVector(ll_reception->source());
                break;
            }
            case GossipType_FacilityLayerReception:
            {
                rosGossipMessage.is_facilitylayer_rx = true;
                auto fl_reception = gossipMessage->gossip_as_FacilityLayerReception();
                auto fl_msg = fl_reception->msg();
                ROS_INFO("FacilityLayerReception: ");
                if (fl_msg->cam_msg() != nullptr)
                {
                    auto cam_msg = fl_msg->cam_msg();

                    rosGossipMessage.facilitylayer_rx.msg.cam_msg.generation_delta_time = cam_msg->generation_delta_time();

                    auto header = cam_msg->header();
                    if (header != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.header.message_id = header->message_id();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.header.protocol_version = header->protocol_version();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.header.station_id = header->station_id();
                    }

                    auto station_type = cam_msg->station_type();
                    rosGossipMessage.facilitylayer_rx.msg.cam_msg.station_type.statype = station_type;

                    auto reference_position = cam_msg->reference_position();
                    if (reference_position != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.latitude = reference_position->latitude();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.longitude = reference_position->longitude();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.position_confidence.semi_major_confidence = reference_position->position_confidence()->semi_major_confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.position_confidence.semi_major_orientation = reference_position->position_confidence()->semi_major_orientation();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.position_confidence.semi_minor_confidence = reference_position->position_confidence()->semi_minor_confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.altitude.value = reference_position->altitude()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.reference_position.altitude.confidence = reference_position->altitude()->confidence();
                    }

                    auto high_frequency_container = cam_msg->high_frequency_container();
                    if (high_frequency_container != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.heading.value = high_frequency_container->heading()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.heading.confidence = high_frequency_container->heading()->confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.speed.value = high_frequency_container->speed()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.speed.confidence = high_frequency_container->speed()->confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.drive_direction.type = high_frequency_container->drive_direction();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.vehicle_length.value = high_frequency_container->vehicle_length()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.vehicle_length.veh_len_conf_ind.type = high_frequency_container->vehicle_length()->veh_len_conf_ind();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.vehicle_width.value = high_frequency_container->vehicle_width()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.longitudinal_acceleration.value = high_frequency_container->longitudinal_acceleration()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.longitudinal_acceleration.confidence = high_frequency_container->longitudinal_acceleration()->confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.curvature.value = high_frequency_container->curvature()->value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.curvature.confidence = high_frequency_container->curvature()->confidence();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.curvature_calculation_mode.type = high_frequency_container->curvature_calculation_mode();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.yaw_rate.yawratevalue = high_frequency_container->yaw_rate()->yaw_rate_value();
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.high_frequency_container.yaw_rate.yawrateconfidence.type = high_frequency_container->yaw_rate()->yaw_rate_confidence();
                    }

                    auto low_frequency_container = cam_msg->low_frequency_container();
                    if (low_frequency_container != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cam_msg.low_frequency_container.temp2 = low_frequency_container->temp2();
                    }
                }
                if (fl_msg->cpm_msg() != nullptr)
                {
                    auto cpm_msg = fl_msg->cpm_msg();
                    // Access the fields of the CPMessage
                    auto generation_delta_time = cpm_msg->generation_delta_time();
                    rosGossipMessage.facilitylayer_rx.msg.cpm_msg.generation_delta_time = generation_delta_time;

                    auto header = cpm_msg->header();
                    auto mgmt_container = cpm_msg->mgmt_container();
                    auto cpm_payload = cpm_msg->cpm_payload();

                    // Access the fields of the ItsPduHeader
                    if (header != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cpm_msg.header.message_id = header->message_id();
                        rosGossipMessage.facilitylayer_rx.msg.cpm_msg.header.protocol_version = header->protocol_version();
                        rosGossipMessage.facilitylayer_rx.msg.cpm_msg.header.station_id = header->station_id();
                    }

                    // Access the fields of the ManagementContainer
                    if (mgmt_container != nullptr)
                    {
                        rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referenceTime = mgmt_container->reference_time();

                        auto reference_position = mgmt_container->reference_position();
                        if (reference_position != nullptr)
                        {
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referencePosition.latitude = reference_position->latitude();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referencePosition.longitude = reference_position->longitude();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referencePosition.position_confidence.semi_major_confidence = reference_position->position_confidence()->semi_major_confidence();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referencePosition.position_confidence.semi_major_orientation = reference_position->position_confidence()->semi_major_orientation();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.referencePosition.position_confidence.semi_minor_confidence = reference_position->position_confidence()->semi_minor_confidence();
                        }

                        auto segmentation_info = mgmt_container->segmentation_info();
                        if (segmentation_info != nullptr)
                        {
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.segmentationInfo.totalMsgNo = segmentation_info->total_msg_no();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.segmentationInfo.thisMsgNo = segmentation_info->this_msg_no();
                        }

                        auto message_rate_range = mgmt_container->message_rate_range();
                        if (message_rate_range != nullptr)
                        {
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.messageRateRange.messageRateMax.exponent = message_rate_range->message_rate_max()->exponent();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.messageRateRange.messageRateMax.mantissa = message_rate_range->message_rate_max()->mantissa();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.messageRateRange.messageRateMin.exponent = message_rate_range->message_rate_min()->exponent();
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.messageRateRange.messageRateMin.mantissa = message_rate_range->message_rate_min()->mantissa();
                        }

                        auto station_type = mgmt_container->station_type();
                        rosGossipMessage.facilitylayer_rx.msg.cpm_msg.mgmt_cntnr.stationtype.statype = station_type;
                    }

                    // Access the fields of the CpmPayload
                    if (cpm_payload != nullptr)
                    {

                        auto originating_stations_container = cpm_payload->originating_stations_container();
                        auto sensor_information_container = cpm_payload->sensor_information_container();
                        auto perception_region_container = cpm_payload->perception_region_container();
                        auto perceived_object_container = cpm_payload->perceived_object_container();

                        if (originating_stations_container != nullptr)
                        {
                            auto originating_vehicle_container = originating_stations_container->originating_vehicle_container();
                            if (originating_vehicle_container != nullptr)
                            {
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.orientationAngle.wgsAngleValue = originating_vehicle_container->orientation_angle()->wgs_angle_value();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.orientationAngle.wgs84AngleConfidence = originating_vehicle_container->orientation_angle()->wgs84_angle_confidence();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.pitchAngle.cartesian_value = originating_vehicle_container->pitch_angle()->cartesian_value();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.pitchAngle.cartesian_angle_confidence = originating_vehicle_container->pitch_angle()->cartesian_angle_confidence();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.rollAngle.cartesian_value = originating_vehicle_container->roll_angle()->cartesian_value();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.rollAngle.cartesian_angle_confidence = originating_vehicle_container->roll_angle()->cartesian_angle_confidence();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_veh_cntr.trailerDataset = originating_vehicle_container->trailer_dataset();
                            }

                            auto originating_rsu_container = originating_stations_container->originating_rsu_container();
                            if (originating_rsu_container != nullptr)
                            {
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_rsu_cntr.roadsegment.region = originating_rsu_container->road_segment()->region();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_rsu_cntr.roadsegment.id = originating_rsu_container->road_segment()->id();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_rsu_cntr.intersection.region = originating_rsu_container->intersection()->region();
                                rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.org_stn_cntr.org_rsu_cntr.intersection.id = originating_rsu_container->intersection()->id();
                            }
                        }

                        if (sensor_information_container != nullptr)
                        {
                            auto sensor_information_vector = sensor_information_container->sensor_information();

                            if (sensor_information_vector != nullptr)
                            {
                                for (auto sensor_information : *sensor_information_vector)
                                {
                                    cpm_interfaces::SensorInformation rosSensorInformation;
                                    rosSensorInformation.sensorID = sensor_information->sensor_id();
                                    rosSensorInformation.perception_region_conf = sensor_information->perception_region_confidence();
                                    rosSensorInformation.shadowingapplies = sensor_information->shadowing_applies();
                                    rosSensorInformation.sensor_type.type = sensor_information->sensor_type();

                                    auto perception_region_shape = sensor_information->perception_region_shape();
                                    if (perception_region_shape != nullptr)
                                    {
                                        rosSensorInformation.perception_region_shape.rectangular.center_point.x_cord = perception_region_shape->rectangular()->center_point()->x_cord();
                                        rosSensorInformation.perception_region_shape.rectangular.center_point.y_cord = perception_region_shape->rectangular()->center_point()->y_cord();
                                        rosSensorInformation.perception_region_shape.rectangular.center_point.z_cord = perception_region_shape->rectangular()->center_point()->z_cord();
                                        rosSensorInformation.perception_region_shape.rectangular.semiLength = perception_region_shape->rectangular()->semi_length();
                                        rosSensorInformation.perception_region_shape.rectangular.semiBreadth = perception_region_shape->rectangular()->semi_breadth();
                                        rosSensorInformation.perception_region_shape.rectangular.orientation = perception_region_shape->rectangular()->orientation();
                                        rosSensorInformation.perception_region_shape.rectangular.height = perception_region_shape->rectangular()->height();

                                        rosSensorInformation.perception_region_shape.circular.shapeRefPoint.x_cord = perception_region_shape->circular()->shape_ref_point()->x_cord();
                                        rosSensorInformation.perception_region_shape.circular.shapeRefPoint.y_cord = perception_region_shape->circular()->shape_ref_point()->y_cord();
                                        rosSensorInformation.perception_region_shape.circular.shapeRefPoint.z_cord = perception_region_shape->circular()->shape_ref_point()->z_cord();
                                        rosSensorInformation.perception_region_shape.circular.radius = perception_region_shape->circular()->radius();
                                        rosSensorInformation.perception_region_shape.circular.height = perception_region_shape->circular()->height();

                                        rosSensorInformation.perception_region_shape.polygonal.shapeReferencePoint.x_cord = perception_region_shape->polygonal()->shape_reference_point()->x_cord();
                                        rosSensorInformation.perception_region_shape.polygonal.shapeReferencePoint.y_cord = perception_region_shape->polygonal()->shape_reference_point()->y_cord();
                                        rosSensorInformation.perception_region_shape.polygonal.shapeReferencePoint.z_cord = perception_region_shape->polygonal()->shape_reference_point()->z_cord();
                                        auto polygon = perception_region_shape->polygonal()->polygon();
                                        if (polygon != nullptr)
                                        {
                                            for (auto point : *polygon)
                                            {
                                                cpm_interfaces::CartesianPosition3d rosPoint3D;
                                                rosPoint3D.x_cord = point->x_cord();
                                                rosPoint3D.y_cord = point->y_cord();
                                                rosPoint3D.z_cord = point->z_cord();
                                                rosSensorInformation.perception_region_shape.polygonal.polygon.push_back(rosPoint3D);
                                            }
                                        }
                                        rosSensorInformation.perception_region_shape.polygonal.height = perception_region_shape->polygonal()->height();

                                        rosSensorInformation.perception_region_shape.elliptical.shapeReferencePoint.x_cord = perception_region_shape->elliptical()->shape_reference_point()->x_cord();
                                        rosSensorInformation.perception_region_shape.elliptical.shapeReferencePoint.y_cord = perception_region_shape->elliptical()->shape_reference_point()->y_cord();
                                        rosSensorInformation.perception_region_shape.elliptical.shapeReferencePoint.z_cord = perception_region_shape->elliptical()->shape_reference_point()->z_cord();
                                        rosSensorInformation.perception_region_shape.elliptical.semiMajorAxisLength = perception_region_shape->elliptical()->semi_major_axis_length();
                                        rosSensorInformation.perception_region_shape.elliptical.semiMinorAxisLength = perception_region_shape->elliptical()->semi_minor_axis_length();
                                        rosSensorInformation.perception_region_shape.elliptical.orientation = perception_region_shape->elliptical()->orientation();
                                        rosSensorInformation.perception_region_shape.elliptical.height = perception_region_shape->elliptical()->height();

                                        rosSensorInformation.perception_region_shape.radial.sharedRefPoint.x_cord = perception_region_shape->radial()->shared_ref_point()->x_cord();
                                        rosSensorInformation.perception_region_shape.radial.sharedRefPoint.y_cord = perception_region_shape->radial()->shared_ref_point()->y_cord();
                                        rosSensorInformation.perception_region_shape.radial.sharedRefPoint.z_cord = perception_region_shape->radial()->shared_ref_point()->z_cord();
                                        rosSensorInformation.perception_region_shape.radial.range = perception_region_shape->radial()->range();
                                        rosSensorInformation.perception_region_shape.radial.stationaryHorizontalOpeningAngleStart = perception_region_shape->radial()->stationary_horizontal_opening_angle_start();
                                        rosSensorInformation.perception_region_shape.radial.stationaryHorizontalOpeningAngleEnd = perception_region_shape->radial()->stationary_horizontal_opening_angle_end();
                                        rosSensorInformation.perception_region_shape.radial.verticalOpeningAngleStart = perception_region_shape->radial()->vertical_opening_angle_start();
                                        rosSensorInformation.perception_region_shape.radial.verticalOpeningAngleEnd = perception_region_shape->radial()->vertical_opening_angle_end();

                                        rosSensorInformation.perception_region_shape.radialShapes.refPointID = perception_region_shape->radial_shapes()->ref_point_id();
                                        rosSensorInformation.perception_region_shape.radialShapes.x_cord = perception_region_shape->radial_shapes()->x_coord();
                                        rosSensorInformation.perception_region_shape.radialShapes.y_cord = perception_region_shape->radial_shapes()->y_coord();
                                        rosSensorInformation.perception_region_shape.radialShapes.z_cord = perception_region_shape->radial_shapes()->z_coord();
                                        auto radial_shapes_list = perception_region_shape->radial_shapes()->radial_shapes_list();
                                        if (radial_shapes_list != nullptr)
                                        {
                                            for (auto radial_shape : *radial_shapes_list)
                                            {
                                                cpm_interfaces::RadialShapeDetails rosRadialShapeDetails;
                                                rosRadialShapeDetails.range = radial_shape->range();
                                                rosRadialShapeDetails.horizontalOpeningAngleStart = radial_shape->horizontal_opening_angle_start();
                                                rosRadialShapeDetails.horizontalOpeningAngleEnd = radial_shape->horizontal_opening_angle_end();
                                                rosRadialShapeDetails.verticalOpeningAngleStart = radial_shape->vertical_opening_angle_start();
                                                rosRadialShapeDetails.verticalOpeningAngleEnd = radial_shape->vertical_opening_angle_end();
                                                rosSensorInformation.perception_region_shape.radialShapes.radialShapesList.push_back(rosRadialShapeDetails);
                                            }
                                        }
                                    }
                                    rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.snsr_info_cntr.snsr_infrmtn.push_back(rosSensorInformation);
                                }
                            }
                        }

                        if (perception_region_container != nullptr)
                        {
                            auto perception_region_list = perception_region_container->perception_region_list();
                            if (perception_region_list != nullptr)
                            {
                                for (auto perception_region : *perception_region_list)
                                {
                                    cpm_interfaces::PerceptionRegion rosPerceptionRegion;
                                    rosPerceptionRegion.measurementDeltaTime = perception_region->measurement_delta_time();
                                    rosPerceptionRegion.perceptionRegionConfidence = perception_region->perception_region_confidence();

                                    auto perception_region_shape = perception_region->perception_region_shape();
                                    if (perception_region_shape != nullptr)
                                    {
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.center_point.x_cord = perception_region_shape->rectangular()->center_point()->x_cord();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.center_point.y_cord = perception_region_shape->rectangular()->center_point()->y_cord();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.center_point.z_cord = perception_region_shape->rectangular()->center_point()->z_cord();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.semiLength = perception_region_shape->rectangular()->semi_length();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.semiBreadth = perception_region_shape->rectangular()->semi_breadth();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.orientation = perception_region_shape->rectangular()->orientation();
                                        rosPerceptionRegion.perceptionRegionShape.rectangular.height = perception_region_shape->rectangular()->height();

                                        rosPerceptionRegion.perceptionRegionShape.circular.shapeRefPoint.x_cord = perception_region_shape->circular()->shape_ref_point()->x_cord();
                                        rosPerceptionRegion.perceptionRegionShape.circular.shapeRefPoint.y_cord = perception_region_shape->circular()->shape_ref_point()->y_cord();
                                        rosPerceptionRegion.perceptionRegionShape.circular.shapeRefPoint.z_cord = perception_region_shape->circular()->shape_ref_point()->z_cord();
                                        rosPerceptionRegion.perceptionRegionShape.circular.radius = perception_region_shape->circular()->radius();
                                        rosPerceptionRegion.perceptionRegionShape.circular.height = perception_region_shape->circular()->height();

                                        rosPerceptionRegion.perceptionRegionShape.polygonal.shapeReferencePoint.x_cord = perception_region_shape->polygonal()->shape_reference_point()->x_cord();
                                        rosPerceptionRegion.perceptionRegionShape.polygonal.shapeReferencePoint.y_cord = perception_region_shape->polygonal()->shape_reference_point()->y_cord();
                                        rosPerceptionRegion.perceptionRegionShape.polygonal.shapeReferencePoint.z_cord = perception_region_shape->polygonal()->shape_reference_point()->z_cord();
                                        auto polygon = perception_region_shape->polygonal()->polygon();
                                        if (polygon != nullptr)
                                        {
                                            for (auto point : *polygon)
                                            {
                                                cpm_interfaces::CartesianPosition3d rosPoint3D;
                                                rosPoint3D.x_cord = point->x_cord();
                                                rosPoint3D.y_cord = point->y_cord();
                                                rosPoint3D.z_cord = point->z_cord();
                                                rosPerceptionRegion.perceptionRegionShape.polygonal.polygon.push_back(rosPoint3D);
                                            }
                                        }
                                        rosPerceptionRegion.perceptionRegionShape.polygonal.height = perception_region_shape->polygonal()->height();

                                        rosPerceptionRegion.perceptionRegionShape.elliptical.shapeReferencePoint.x_cord = perception_region_shape->elliptical()->shape_reference_point()->x_cord();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.shapeReferencePoint.y_cord = perception_region_shape->elliptical()->shape_reference_point()->y_cord();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.shapeReferencePoint.z_cord = perception_region_shape->elliptical()->shape_reference_point()->z_cord();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.semiMajorAxisLength = perception_region_shape->elliptical()->semi_major_axis_length();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.semiMinorAxisLength = perception_region_shape->elliptical()->semi_minor_axis_length();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.orientation = perception_region_shape->elliptical()->orientation();
                                        rosPerceptionRegion.perceptionRegionShape.elliptical.height = perception_region_shape->elliptical()->height();

                                        rosPerceptionRegion.perceptionRegionShape.radial.sharedRefPoint.x_cord = perception_region_shape->radial()->shared_ref_point()->x_cord();
                                        rosPerceptionRegion.perceptionRegionShape.radial.sharedRefPoint.y_cord = perception_region_shape->radial()->shared_ref_point()->y_cord();
                                        rosPerceptionRegion.perceptionRegionShape.radial.sharedRefPoint.z_cord = perception_region_shape->radial()->shared_ref_point()->z_cord();
                                        rosPerceptionRegion.perceptionRegionShape.radial.range = perception_region_shape->radial()->range();
                                        rosPerceptionRegion.perceptionRegionShape.radial.stationaryHorizontalOpeningAngleStart = perception_region_shape->radial()->stationary_horizontal_opening_angle_start();
                                        rosPerceptionRegion.perceptionRegionShape.radial.stationaryHorizontalOpeningAngleEnd = perception_region_shape->radial()->stationary_horizontal_opening_angle_end();
                                        rosPerceptionRegion.perceptionRegionShape.radial.verticalOpeningAngleStart = perception_region_shape->radial()->vertical_opening_angle_start();
                                        rosPerceptionRegion.perceptionRegionShape.radial.verticalOpeningAngleEnd = perception_region_shape->radial()->vertical_opening_angle_end();

                                        rosPerceptionRegion.perceptionRegionShape.radialShapes.refPointID = perception_region_shape->radial_shapes()->ref_point_id();
                                        rosPerceptionRegion.perceptionRegionShape.radialShapes.x_cord = perception_region_shape->radial_shapes()->x_coord();
                                        rosPerceptionRegion.perceptionRegionShape.radialShapes.y_cord = perception_region_shape->radial_shapes()->y_coord();
                                        rosPerceptionRegion.perceptionRegionShape.radialShapes.z_cord = perception_region_shape->radial_shapes()->z_coord();
                                        auto radial_shapes_list = perception_region_shape->radial_shapes()->radial_shapes_list();
                                        if (radial_shapes_list != nullptr)
                                        {
                                            for (auto radial_shape : *radial_shapes_list)
                                            {
                                                cpm_interfaces::RadialShapeDetails rosRadialShapeDetails;
                                                rosRadialShapeDetails.range = radial_shape->range();
                                                rosRadialShapeDetails.horizontalOpeningAngleStart = radial_shape->horizontal_opening_angle_start();
                                                rosRadialShapeDetails.horizontalOpeningAngleEnd = radial_shape->horizontal_opening_angle_end();
                                                rosRadialShapeDetails.verticalOpeningAngleStart = radial_shape->vertical_opening_angle_start();
                                                rosRadialShapeDetails.verticalOpeningAngleEnd = radial_shape->vertical_opening_angle_end();
                                                rosPerceptionRegion.perceptionRegionShape.radialShapes.radialShapesList.push_back(rosRadialShapeDetails);
                                            }
                                        }
                                    }
                                    rosPerceptionRegion.shadowingApplies = perception_region->shadowing_applies();
                                    // auto sensor_id_list = perception_region->sensor_id_list();
                                    rosPerceptionRegion.sensorIdList = ConvertFlatbuffersVector(perception_region->sensor_id_list());
                                    // for (auto sensor_id : *sensor_id_list)
                                    // {
                                    //     rosPerceptionRegion.sensorIdList.push_back(sensor_id);
                                    // }
                                    rosPerceptionRegion.numberOfPerceivedObjects = perception_region->number_of_perceived_objects();
                                    rosPerceptionRegion.perceivedObjectIds = perception_region->perceived_object_ids();

                                    rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.prcptn_rgn_cntr.prcpn_rgn_list.push_back(rosPerceptionRegion);
                                }
                            }
                        }

                        if (perceived_object_container != nullptr)
                        {
                            rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.prcvd_obj_cntr.numberOfPerceivedObjects = perceived_object_container->number_of_perceived_objects();
                            auto perceived_objects_vector = perceived_object_container->perceived_objects();
                            if (perceived_objects_vector != nullptr)
                            {
                                for (auto perceived_object : *perceived_object_container->perceived_objects())
                                {
                                    cpm_interfaces::PerceivedObject rosPerceivedObject;
                                    rosPerceivedObject.objectID = perceived_object->object_id();
                                    rosPerceivedObject.measurementDeltaTime = perceived_object->measurement_delta_time();
                                    rosPerceivedObject.objectAge = perceived_object->object_age();
                                    rosPerceivedObject.objectPerceptionQuality = perceived_object->object_perception_quality();
                                    rosPerceivedObject.sensorIdList = ConvertFlatbuffersVector(perceived_object->sensor_id_list());
                                    // for (auto sensor_id : *perceived_object->sensor_id_list())
                                    // {
                                    //     rosPerceivedObject.sensorIdList.push_back(sensor_id);
                                    // }
                                    auto position = perceived_object->position();
                                    if (position != nullptr)
                                    {
                                        rosPerceivedObject.position.x_cord.value = position->x_cord()->value();
                                        rosPerceivedObject.position.x_cord.confidence = position->x_cord()->confidence();

                                        rosPerceivedObject.position.y_cord.value = position->y_cord()->value();
                                        rosPerceivedObject.position.y_cord.confidence = position->y_cord()->confidence();
                                        if (position->z_cord() != nullptr)
                                        {
                                            rosPerceivedObject.position.z_cord.value = position->z_cord()->value();
                                            rosPerceivedObject.position.z_cord.confidence = position->z_cord()->confidence();
                                        }
                                    }
                                    auto velocity = perceived_object->velocity();
                                    if (velocity != nullptr)
                                    {
                                        rosPerceivedObject.velocity.polarVelocity.velocityMagnitude.value = velocity->polar_velocity()->velocity_magnitude()->value();
                                        rosPerceivedObject.velocity.polarVelocity.velocityMagnitude.confidence = velocity->polar_velocity()->velocity_magnitude()->confidence();
                                        rosPerceivedObject.velocity.polarVelocity.velocityDirection.cartesian_value = velocity->polar_velocity()->velocity_direction()->cartesian_value();
                                        rosPerceivedObject.velocity.polarVelocity.velocityDirection.cartesian_angle_confidence = velocity->polar_velocity()->velocity_direction()->cartesian_angle_confidence();
                                        rosPerceivedObject.velocity.polarVelocity.zVelocity.vel_comp_value = velocity->polar_velocity()->z_velocity()->vel_comp_value();
                                        rosPerceivedObject.velocity.polarVelocity.zVelocity.speed_confidence = velocity->polar_velocity()->z_velocity()->speed_confidence();
                                    }
                                    auto acceleration = perceived_object->acceleration();
                                    if (acceleration != nullptr)
                                    {
                                        rosPerceivedObject.acceleration.polarAcceleration.accelerationMagnitude.accelerationMagnitudeValue = acceleration->polar_acceleration()->acceleration_magnitude()->acceleration_magnitude_value();
                                        rosPerceivedObject.acceleration.polarAcceleration.accelerationMagnitude.accelerationConfidence = acceleration->polar_acceleration()->acceleration_magnitude()->acceleration_confidence();
                                        rosPerceivedObject.acceleration.polarAcceleration.accelerationDirection.cartesian_value = acceleration->polar_acceleration()->acceleration_direction()->cartesian_value();
                                        rosPerceivedObject.acceleration.polarAcceleration.accelerationDirection.cartesian_angle_confidence = acceleration->polar_acceleration()->acceleration_direction()->cartesian_angle_confidence();
                                        if (acceleration->polar_acceleration()->z_acceleration() != nullptr)
                                        {
                                            rosPerceivedObject.acceleration.polarAcceleration.zAcceleration.value = acceleration->polar_acceleration()->z_acceleration()->value();
                                            rosPerceivedObject.acceleration.polarAcceleration.zAcceleration.confidence = acceleration->polar_acceleration()->z_acceleration()->confidence();
                                        }
                                    }

                                    auto angles = perceived_object->angles();
                                    if (angles != nullptr)
                                    {
                                        rosPerceivedObject.angles.zAngle.cartesian_value = angles->z_angle()->cartesian_value();
                                        rosPerceivedObject.angles.zAngle.cartesian_angle_confidence = angles->z_angle()->cartesian_angle_confidence();

                                        if (angles->x_angle() != nullptr)
                                        {
                                            rosPerceivedObject.angles.xAngle.cartesian_value = angles->x_angle()->cartesian_value();
                                            rosPerceivedObject.angles.xAngle.cartesian_angle_confidence = angles->x_angle()->cartesian_angle_confidence();
                                        }
                                        if (angles->y_angle() != nullptr)
                                        {
                                            rosPerceivedObject.angles.yAngle.cartesian_value = angles->y_angle()->cartesian_value();
                                            rosPerceivedObject.angles.yAngle.cartesian_angle_confidence = angles->y_angle()->cartesian_angle_confidence();
                                        }
                                    }

                                    auto z_angular_velocity = perceived_object->z_angular_velocity();
                                    if (z_angular_velocity != nullptr)
                                    {
                                        rosPerceivedObject.zAngularVelocity.value = z_angular_velocity->value();
                                        rosPerceivedObject.zAngularVelocity.confidence.confi = z_angular_velocity->confidence();
                                    }
                                    auto lower_triangular_correlation_matrices = perceived_object->lower_triangular_correlation_matrices();
                                    if (lower_triangular_correlation_matrices != nullptr)
                                    {
                                        for (auto lower_triangular_correlation_matrix : *lower_triangular_correlation_matrices)
                                        {
                                            cpm_interfaces::LowerTriangularPositiveSemidefiniteMatrix rosLowerTriangularPositiveSemidefiniteMatrix;

                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.xPosition = lower_triangular_correlation_matrix->components_included_in_the_matrix()->x_position();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.yPosition = lower_triangular_correlation_matrix->components_included_in_the_matrix()->y_position();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.zPosition = lower_triangular_correlation_matrix->components_included_in_the_matrix()->z_position();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.xVelocityOrVelocityMagnitude = lower_triangular_correlation_matrix->components_included_in_the_matrix()->x_velocity_or_velocity_magnitude();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.yVelocityOrVelocityDirection = lower_triangular_correlation_matrix->components_included_in_the_matrix()->y_velocity_or_velocity_direction();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.zSpeed = lower_triangular_correlation_matrix->components_included_in_the_matrix()->z_speed();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.xAccelOrAccelMagnitude = lower_triangular_correlation_matrix->components_included_in_the_matrix()->x_accel_or_accel_magnitude();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.yAccelOrAccelDirection = lower_triangular_correlation_matrix->components_included_in_the_matrix()->y_accel_or_accel_direction();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.zAcceleration = lower_triangular_correlation_matrix->components_included_in_the_matrix()->z_acceleration();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.zAngle = lower_triangular_correlation_matrix->components_included_in_the_matrix()->z_angle();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.yAngle = lower_triangular_correlation_matrix->components_included_in_the_matrix()->y_angle();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.xAngle = lower_triangular_correlation_matrix->components_included_in_the_matrix()->x_angle();
                                            rosLowerTriangularPositiveSemidefiniteMatrix.componentsIncludedIntheMatrix.zAngularVelocity = lower_triangular_correlation_matrix->components_included_in_the_matrix()->z_angular_velocity();

                                            auto columns = lower_triangular_correlation_matrix->matrix()->columns();
                                            if (columns != nullptr)
                                            {
                                                for (auto column : *columns)
                                                {
                                                    cpm_interfaces::CorrelationColumn rosCorrelationColumn;
                                                    rosCorrelationColumn.value = ConvertFlatbuffersVector(column->value());
                                                    // for (auto value : *column->value())
                                                    // {
                                                    //     rosCorrelationColumn.value.push_back(value);
                                                    // }
                                                    rosLowerTriangularPositiveSemidefiniteMatrix.matrix.columns.push_back(rosCorrelationColumn);
                                                }
                                            }
                                            rosPerceivedObject.lowerTriangularCorrelationMatrices.push_back(rosLowerTriangularPositiveSemidefiniteMatrix);
                                        }
                                    }
                                    auto object_dimension_z = perceived_object->object_dimension_z();
                                    if (object_dimension_z != nullptr)
                                    {
                                        rosPerceivedObject.objectDimensionZ.value = object_dimension_z->value();
                                        rosPerceivedObject.objectDimensionZ.confidence = object_dimension_z->confidence();
                                    }

                                    auto object_dimension_y = perceived_object->object_dimension_y();
                                    if (object_dimension_y != nullptr)
                                    {
                                        rosPerceivedObject.objectDimensionY.value = object_dimension_y->value();
                                        rosPerceivedObject.objectDimensionY.confidence = object_dimension_y->confidence();
                                    }

                                    auto object_dimension_x = perceived_object->object_dimension_x();
                                    if (object_dimension_x != nullptr)
                                    {
                                        rosPerceivedObject.objectDimensionX.value = object_dimension_x->value();
                                        rosPerceivedObject.objectDimensionX.confidence = object_dimension_x->confidence();
                                    }

                                    auto classification = perceived_object->classification();
                                    if (classification != nullptr)
                                    {
                                        for (auto object_class_with_confidence : *classification)
                                        {
                                            cpm_interfaces::ObjectClassWithConfidence rosObjectClassWithConfidence;
                                            rosObjectClassWithConfidence.confidence = object_class_with_confidence->confidence();
                                            rosObjectClassWithConfidence.objectClass.vehicleSubClass.type = object_class_with_confidence->object_class()->vehicle_sub_class();
                                            rosObjectClassWithConfidence.objectClass.vruSubClass.pedestrian.type = object_class_with_confidence->object_class()->vru_sub_class()->pedestrian();
                                            rosObjectClassWithConfidence.objectClass.vruSubClass.bicyclistAndLightVruVehicle.type = object_class_with_confidence->object_class()->vru_sub_class()->bicyclist_and_light_vru_vehicle();
                                            rosObjectClassWithConfidence.objectClass.vruSubClass.motorcyclist.type = object_class_with_confidence->object_class()->vru_sub_class()->motorcyclist();
                                            rosObjectClassWithConfidence.objectClass.vruSubClass.animal.type = object_class_with_confidence->object_class()->vru_sub_class()->animal();

                                            rosPerceivedObject.classification.push_back(rosObjectClassWithConfidence);
                                        }
                                    }

                                    auto map_position = perceived_object->map_position();
                                    if (map_position != nullptr)
                                    {
                                        rosPerceivedObject.mapPosition.mapReference.roadsegment.region = map_position->map_reference()->road_segment()->region();
                                        rosPerceivedObject.mapPosition.mapReference.roadsegment.id = map_position->map_reference()->road_segment()->id();
                                        rosPerceivedObject.mapPosition.mapReference.intersection.region = map_position->map_reference()->intersection()->region();
                                        rosPerceivedObject.mapPosition.mapReference.intersection.id = map_position->map_reference()->intersection()->id();

                                        rosPerceivedObject.mapPosition.laneId = map_position->lane_id();
                                        rosPerceivedObject.mapPosition.connectionId = map_position->connection_id();

                                        rosPerceivedObject.mapPosition.longitudinalLanePosition.longitudinalLanePositionValue = map_position->longitudinal_lane_position()->longitudinal_lane_position_value();
                                        rosPerceivedObject.mapPosition.longitudinalLanePosition.longitudinalLanePositionConfidence = map_position->longitudinal_lane_position()->longitudinal_lane_position_confidence();
                                    }

                                    rosGossipMessage.facilitylayer_rx.msg.cpm_msg.cpm_payload.prcvd_obj_cntr.perceivedObjects.push_back(rosPerceivedObject);
                                }
                            }
                        }
                    }
                }

                break;
            }
            default:
                ROS_INFO("Unknown gossip message type");
            }
            publisher_.publish(rosGossipMessage);
            start_receive();
        }
        else
        {
            ROS_ERROR("Error in receiving data: %s", error.message().c_str());
        }
    }

private:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    boost::asio::io_context io_context_;
    unique_ptr<udp::socket> socket_;
    udp::endpoint remote_endpoint_;
    array<char, 1024> recv_buffer_;
    unique_ptr<thread> io_thread_;
};

/**
 * @brief Main function of the car2x unit node
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "car2x_unit");

    ros::NodeHandle nh("~"); // "~" means private NodeHandle
    std::string ip_address;
    int local_port, remote_port;

    // The second parameter to param() is the default value to use if the parameter was not set
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("local_port", local_port, 12346);
    nh.param<int>("remote_port", remote_port, 12345);

    ROS_INFO("IP Address: %s", ip_address.c_str());
    ROS_INFO("Local Port: %d", local_port);
    ROS_INFO("Remote Port: %d", remote_port);

    Car2xUnit node(ip_address, local_port, remote_port);
    ros::spin();
    return 0;
}
