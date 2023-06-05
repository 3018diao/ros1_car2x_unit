#include <ros/ros.h>
#include "std_msgs/String.h"
// #include "cpm_interface.pb.h"
#include <boost/asio.hpp>
#include <ros/message_operations.h>
#include <sstream>
#include "cpm_interfaces/PerceivedObjectContainer.h"
#include "gossip_msg_generated.h"
#include "cpm_interface_generated.h"

using boost::asio::ip::udp;
using namespace std;
using namespace Gossip;
using namespace cpm_interfaces;

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
        ROS_INFO("----------------------------------------------------------------------------------");
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
            case GossipType_ChannelBusyRatio:
            {
                auto busy_ratio = msg->gossip_as_ChannelBusyRatio();
                ROS_INFO("receive: ChannelBusyRatio: \n  busy=%u\n  total=%u", busy_ratio->busy(), busy_ratio->total());
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
                auto cp_message = fl_reception->msg_as_CPMessage();
                ROS_INFO("FacilityLayerReception received. ItsPduHeader: \n  protocol_version=%u\n  message_id=%u\n  station_id=%u\n  generation_delta_time=%lu",
                         cp_message->header()->protocol_version(), cp_message->header()->message_id(), cp_message->header()->station_id(), cp_message->generation_delta_time());
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
