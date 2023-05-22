#include <ros/ros.h>
#include "std_msgs/String.h"
#include "cpm_interface.pb.h"
#include <boost/asio.hpp>
#include <ros/message_operations.h>
#include <sstream>
#include "car2x_unit/PerceivedObjectContainer.h"

using boost::asio::ip::udp;
using namespace std;
class UDPListenerNode
{
public:
    UDPListenerNode()
    {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<std_msgs::String>("udp_data", 10);
        subscriber_ = nh.subscribe("collective_perception", 1000, &UDPListenerNode::collectivePerceptionCallback, this);

        socket_ = std::make_unique<udp::socket>(io_context_, udp::endpoint(udp::v4(), 12345));
        start_receive();
        io_thread_ = std::make_unique<std::thread>([this]()
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

    void collectivePerceptionCallback(const car2x_unit::PerceivedObjectContainer::ConstPtr &msg)
    {

        std::ostringstream oss;

        ros::message_operations::Printer<car2x_unit::PerceivedObjectContainer>::stream(oss, "", *msg);

        ROS_INFO("----------------------------------------------------------------------------------\n");
        ROS_INFO("%s", oss.str().c_str());
    }

    void handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (!error)
        {
            CPMessage message;
            if (!message.ParseFromArray(recv_buffer_.data(), bytes_transferred))
            {
                ROS_ERROR("Failed to parse received data");
            }
            else
            {
                std_msgs::String str_msg;
                str_msg.data = message.DebugString();
                publisher_.publish(str_msg);

                ROS_INFO("Received message: %s\n-----------------\n", message.DebugString().c_str());
            }

            start_receive();
        }
    }

private:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    boost::asio::io_context io_context_;
    std::unique_ptr<udp::socket> socket_;
    udp::endpoint remote_endpoint_;
    std::array<char, 1024> recv_buffer_;
    std::unique_ptr<std::thread> io_thread_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener");
    UDPListenerNode node;
    ros::spin();
    return 0;
}
