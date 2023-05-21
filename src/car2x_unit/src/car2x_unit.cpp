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
        ros::NodeHandle nh;                                          // 初始化节点句柄
        publisher_ = nh.advertise<std_msgs::String>("udp_data", 10); // 定义一个发布者，主题名为"udp_data"

        socket_ = std::make_unique<udp::socket>(io_context_, udp::endpoint(udp::v4(), 12345)); // 创建一个UDP套接字，绑定到端口12345

        start_receive(); // 开始接收数据
        io_thread_ = std::make_unique<std::thread>([this]()
                                                   { io_context_.run(); }); // 在新的线程中运行io_context_
    }

    ~UDPListenerNode()
    {
        io_context_.stop(); // 停止io_context_
        if (io_thread_ && io_thread_->joinable())
            io_thread_->join(); // 等待线程结束
    }

private:
    void start_receive()
    {
        socket_->async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&UDPListenerNode::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred)); // 异步接收数据，并绑定回调函数handle_receive
    }

    void handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (!error) // 如果没有错误
        {
            CPMessage message;
            if (!message.ParseFromArray(recv_buffer_.data(), bytes_transferred)) // 解析接收到的数据
            {
                ROS_ERROR("Failed to parse received data"); // 如果解析失败，打印错误信息
            }
            else
            {
                std_msgs::String str_msg;
                str_msg.data = message.DebugString(); // 将接收到的数据转为字符串
                publisher_.publish(str_msg);          // 发布消息

                ROS_INFO("Received message: %s\n-----------------\n", message.DebugString().c_str()); // 打印接收到的消息
            }

            start_receive(); // 继续接收数据
        }
    }

private:
    ros::Publisher publisher_;               // 定义一个发布者
    boost::asio::io_context io_context_;     // 定义一个io_context_
    std::unique_ptr<udp::socket> socket_;    // 定义一个UDP套接字
    udp::endpoint remote_endpoint_;          // 定义一个UDP端点，用于存储发送者的信息
    std::array<char, 1024> recv_buffer_;     // 定义一个接收缓冲区
    std::unique_ptr<std::thread> io_thread_; // 定义一个线程
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener"); // 初始化ROS节点
    UDPListenerNode node;                  // 创建一个UDPListenerNode对象
    ros::spin();                           // 开始ROS事件循环
    return 0;
}
