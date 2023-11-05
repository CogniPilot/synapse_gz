#include "rclcpp/rclcpp.hpp"

#include "clients/gz_client.hpp"
#include "clients/tcp_client.hpp"
#include "synapse_tinyframe/TinyFrame.h"

#include <iostream>
#include <memory>
#include <rclcpp/parameter_value.hpp>

std::atomic<bool> g_stop { false };
std::shared_ptr<TcpClient> g_tcp_client = NULL;
std::shared_ptr<GzClient> g_gz_client = NULL;
std::shared_ptr<TinyFrame> g_tf = NULL;

void signal_handler(int signum)
{
    (void)signum;
    g_stop = true;
}

void ros_entry_point()
{
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
}

void tcp_entry_point()
{
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
}

void gz_entry_point()
{
    while (not g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

class SynapseGz : public rclcpp::Node {
public:
    SynapseGz()
        : Node("synapze_gz")
    {
        this->declare_parameter("host", "127.0.0.1");
        this->declare_parameter("port", 4241);
        this->declare_parameter("vehicle", "mrbuggy3");

        std::string host = this->get_parameter("host").as_string();
        int port = this->get_parameter("port").as_int();
        std::string vehicle = this->get_parameter("vehicle").as_string();

        // create tcp client
        g_tcp_client = std::make_shared<TcpClient>(host, port);

        // create gz client
        g_gz_client = std::make_shared<GzClient>(vehicle, g_tcp_client.get()->tf_);
        g_tcp_client.get()->gz_ = g_gz_client;

        // start threads
        tcp_thread_ = std::make_shared<std::thread>(tcp_entry_point);
        gz_thread_ = std::make_shared<std::thread>(gz_entry_point);
    }
    virtual ~SynapseGz()
    {
        // join threads
        g_stop = true;
        tcp_thread_->join();
        gz_thread_->join();
    }

private:
    std::shared_ptr<std::thread> tcp_thread_;
    std::shared_ptr<std::thread> gz_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseGz>());
    rclcpp::shutdown();

    return 0;
}

// vi: ts=4 sw=4 et
