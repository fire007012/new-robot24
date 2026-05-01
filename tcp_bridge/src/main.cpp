#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include "tcp_bridge/tcp_server.h"
#include "tcp_bridge/ros_handler.h"
#include "tcp_bridge/json_converter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_bridge_node");
    ros::NodeHandle nh("~");

    int port = nh.param<int>("port", 9090);

    ROS_INFO("Starting TCP Bridge Node on port %d", port);

    // Boost ASIO IO context
    boost::asio::io_context io_context;

    // Work guard keeps io_context alive even when there is no pending work
    auto work_guard = boost::asio::make_work_guard(io_context);

    // Create TCP server
    TcpServer tcp_server(io_context, port);

    // Create ROS handler
    RosHandler ros_handler(nh);

    // Connect callbacks
    tcp_server.setMessageCallback([&ros_handler](const std::string& json_msg) {
        ros_handler.handleJsonCommand(json_msg);
    });

    tcp_server.setConnectionCallback([&ros_handler]() {
        ros_handler.onClientConnected();
    });

    ros_handler.setJsonCallback([&tcp_server](const std::string& json_msg) {
        tcp_server.sendMessage(json_msg);
    });

    // Start TCP server
    tcp_server.start();

    // Run IO context in separate thread
    std::thread io_thread([&io_context]() {
        io_context.run();
    });

    // ROS spin
    ros::spin();

    // Cleanup
    work_guard.reset();
    io_context.stop();
    io_thread.join();

    ROS_INFO("TCP Bridge Node shutting down");
    return 0;
}
