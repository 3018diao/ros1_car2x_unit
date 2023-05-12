#include <ros/ros.h>
#include "std_msgs/String.h"
#include "cpm_interface.pb.h"
#include <boost/asio.hpp>

using boost::asio::ip::udp;
using namespace std;

class UDPListenerNode
{
public:
    UDPListenerNode()
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<std_msgs::String>("udp_data", 10);

        socket_ = std::make_unique<udp::socket>(io_context_, udp::endpoint(udp::v4(), 12345));

        receive();
    }

private:
    void receive()
    {
        const int max_length = 1024;
        char buffer[max_length];

        udp::endpoint sender_endpoint;
        size_t length = socket_->receive_from(boost::asio::buffer(buffer, max_length), sender_endpoint);

        CPMessage message;
        if (!message.ParseFromArray(buffer, length))
        {
            ROS_ERROR("Failed to parse received data");
            return;
        }

        auto str_msg = std_msgs::String();
        str_msg.data = message.DebugString();
        publisher_.publish(str_msg);

        ROS_INFO("Received message: %s\n-----------------\n", message.DebugString().c_str());

        receive();
    }

private:
    ros::Publisher publisher_;
    boost::asio::io_context io_context_;
    std::unique_ptr<udp::socket> socket_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener");
    UDPListenerNode node;
    ros::spin();
    return 0;
}
