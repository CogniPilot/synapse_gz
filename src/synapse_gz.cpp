#include "rclcpp/rclcpp.hpp"

#include "clients/gz_client.hpp"
#include "clients/tcp_client.hpp"
#include "synapse_tinyframe/TinyFrame.h"

#include <iostream>
#include <memory>
#include <rclcpp/parameter_value.hpp>

std::atomic<bool> g_stop{false};
std::shared_ptr<TcpClient> g_tcp_client = NULL;
std::shared_ptr<GzClient> g_gz_client = NULL;
std::shared_ptr<TinyFrame> g_tf = NULL;

void signal_handler(int signum) {
    (void)signum;
    g_stop = true;
}

void ros_entry_point()
{
    std::cout << "ros thread started" << std::endl;
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
    std::cout << "ros thread stopped" << std::endl;
}

void tcp_entry_point()
{
    std::cout << "tcp thread started" << std::endl;
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
    std::cout << "tcp thread stopped" << std::endl;
}

void gz_entry_point()
{
    std::cout << "gz thread started" << std::endl;
    while (not g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "gz thread stopped" << std::endl;
}


class SynapseGz : public rclcpp::Node {
  public:
    SynapseGz() : Node("synapze_gz") {
        this->declare_parameter("host", "127.0.0.1");
        this->declare_parameter("port", 4241);

        std::string host = this->get_parameter("host").as_string();
        int port = this->get_parameter("port").as_int();

        // parameters
        std::string prefix = "/world/default/model/elm4/link/sensors/sensor/";

        // create tinyframe
        g_tf = std::make_shared<TinyFrame>(*(TF_Init(TF_MASTER)));

        // create gz client
        g_gz_client = std::make_shared<GzClient>(prefix, g_tf);

        // create tcp client
        g_tcp_client = std::make_shared<TcpClient>(host, port, g_tf);
        g_tcp_client.get()->gz_ = g_gz_client;
        
        // start threads
        tcp_thread_ = std::make_shared<std::thread>(tcp_entry_point);
        gz_thread_ = std::make_shared<std::thread>(gz_entry_point);
    }
    virtual ~SynapseGz() {
        // join threads
        g_stop = true;
        tcp_thread_->join();
        gz_thread_->join();
    }
  private:
    std::shared_ptr<std::thread> tcp_thread_;
    std::shared_ptr<std::thread> gz_thread_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseGz>());
    rclcpp::shutdown();

    return 0;
}

// vi: ts=4 sw=4 et
