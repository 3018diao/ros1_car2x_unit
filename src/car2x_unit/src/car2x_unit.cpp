#include <ros/ros.h>
#include "std_msgs/String.h"
// #include "cpm_interface.pb.h"
#include <boost/asio.hpp>
#include <ros/message_operations.h>
#include <sstream>
#include "cpm_interfaces/PerceivedObjectContainer.h"
#include "etsi_msg_interface_generated.h"
// #include "gossip_msg_generated.h"
// #include "cpm_interface_generated.h"
#include "all_interface_generated.h"

using namespace Gos;

using boost::asio::ip::udp;
using namespace std;
// using namespace Gossip;
// using namespace cpm_interfaces;
using namespace Gos;

class UDPListenerNode
{
public:
    UDPListenerNode()
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<std_msgs::String>("udp_data", 10);
        subscriber_ = nh.subscribe("collective_perception", 1000, &UDPListenerNode::collectivePerceptionCallback, this);

        socket_ = make_unique<udp::socket>(io_context_, udp::endpoint(udp::v4(), 12346));
        start_receive();
        io_thread_ = make_unique<thread>([this]()
                                         { io_context_.run(); });
    }

    ~UDPListenerNode()
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
            boost::bind(&UDPListenerNode::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void collectivePerceptionCallback(const cpm_interfaces::PerceivedObjectContainer::ConstPtr &msg)
    {
        ostringstream oss;
        ros::message_operations::Printer<cpm_interfaces::PerceivedObjectContainer>::stream(oss, "", *msg);
        ROS_INFO("****************");
        ROS_INFO("%s", oss.str().c_str());
    }

    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
    {
        if (!error)
        {
            auto msg = GetGossipMessage(recv_buffer_.data());

            // Check the type of the `gossip` field and handle each type.

            switch (msg->gossip_type())
            {
                // ROS_INFO("");
            case GossipType_ChannelBusyRatio:
            {
                auto busy_ratio = msg->gossip_as_ChannelBusyRatio();
                ROS_INFO("receive: ChannelBusyRatio: \n  busy=%u\n  total=%u", busy_ratio->busy(), busy_ratio->total());
                ROS_INFO("----------------------------------------------------------------------------------");

                break;
            }
            case GossipType_LinkLayerReception:
            {
                auto ll_reception = msg->gossip_as_LinkLayerReception();
                ostringstream source, destination, payload;
                for (auto b : *ll_reception->source())
                    source << to_string(b) << " ";
                for (auto b : *ll_reception->destination())
                    destination << to_string(b) << " ";
                for (auto b : *ll_reception->payload())
                    payload << to_string(b) << " ";

                ROS_INFO("LinkLayerReception: \n  channel=%d\n  power_cbm=%d\n  source=%s\n  destination=%s\n  payload=%s",
                         ll_reception->channel(), ll_reception->power_cbm(), source.str().c_str(), destination.str().c_str(), payload.str().c_str());
                break;
            }
            case GossipType_FacilityLayerReception:
            {
                auto fl_reception = msg->gossip_as_FacilityLayerReception();
                auto fl_msg = fl_reception->msg();
                ROS_INFO("FacilityLayerReception: ");
                if (fl_msg->cam_msg() != nullptr)
                {
                    auto cam_msg = fl_msg->cam_msg();
                    // Now you can access the fields of the CAMessage
                    // For example, if CAMessage has a field named `data`, you can access it like this:
                    // auto data = cam_msg->data();
                    // And then you can print the data or do something else with it.
                }
                if (fl_msg->cpm_msg() != nullptr)
                {
                    auto cpm_msg = fl_msg->cpm_msg();

                    // Access the fields of the CPMessage
                    auto header = cpm_msg->header();
                    auto generation_delta_time = cpm_msg->generation_delta_time();
                    auto mgmt_container = cpm_msg->mgmt_container();
                    auto cpm_payload = cpm_msg->cpm_payload();

                    // Print the fields or do something else with them
                    ROS_INFO("CPMessage: \n  generation_delta_time=%lu", generation_delta_time);

                    // Access the fields of the ItsPduHeader
                    if (header != nullptr)
                    {
                        auto protocol_version = header->protocol_version();
                        auto message_id = header->message_id();
                        auto station_id = header->station_id();

                        // Print the fields or do something else with them
                        ROS_INFO("ItsPduHeader: \n  protocol_version=%u\n  message_id=%u\n  station_id=%u", protocol_version, message_id, station_id);
                    }

                    // Access the fields of the ManagementContainer
                    if (mgmt_container != nullptr)
                    {
                        auto reference_time = mgmt_container->reference_time();
                        auto reference_position = mgmt_container->reference_position();
                        auto segmentation_info = mgmt_container->segmentation_info();
                        auto message_rate_range = mgmt_container->message_rate_range();
                        auto station_type = mgmt_container->station_type();

                        // Print the fields or do something else with them
                        ROS_INFO("ManagementContainer: \n  reference_time=%lu\n  station_type=%u", reference_time, station_type);

                        // Continue with the fields of reference_position, segmentation_info, and message_rate_range...
                    }

                    // Access the fields of the CpmPayload
                    if (cpm_payload != nullptr)
                    {
                        auto originating_stations_container = cpm_payload->originating_stations_container();
                        auto sensor_information_container = cpm_payload->sensor_information_container();
                        auto perception_region_container = cpm_payload->perception_region_container();
                        auto perceived_object_container = cpm_payload->perceived_object_container();

                        // Print the fields or do something else with them
                        ROS_INFO("CpmPayload: \n  originating_stations_container=%p\n  sensor_information_container=%p\n  perception_region_container=%p\n  perceived_object_container=%p", originating_stations_container, sensor_information_container, perception_region_container, perceived_object_container);

                        // Continue with the fields of originating_stations_container, sensor_information_container, perception_region_container, and perceived_object_container...
                        if (originating_stations_container != nullptr)
                        {
                            // Access fields of originating_stations_container...
                        }

                        if (sensor_information_container != nullptr)
                        {
                            // Access fields of sensor_information_container...
                        }

                        if (perception_region_container != nullptr)
                        {
                            // Access fields of perception_region_container...
                        }

                        if (perceived_object_container != nullptr)
                        {
                            for (auto perceived_object : *perceived_object_container->perceived_objects())
                            {
                                auto object_id = perceived_object->object_id();
                                auto measurement_delta_time = perceived_object->measurement_delta_time();
                                // Continue with other fields...

                                // ROS_INFO("PerceivedObject: \n  object_id=%u\n  object_class=%u", object_id, object_class);
                            }
                        }
                    }
                }

                break;
            }
            default:
                ROS_INFO("Unknown gossip message type");
            }

            start_receive();
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener");
    UDPListenerNode node;
    ros::spin();
    return 0;
}
