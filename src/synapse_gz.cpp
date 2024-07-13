#include "rclcpp/rclcpp.hpp"

#include "link/gz_client.hpp"
#include "link/udp_link.hpp"

#include <memory>
#include <rclcpp/parameter_value.hpp>

std::atomic<bool> g_stop { false };
std::shared_ptr<UDPLink> g_udp_link = NULL;
std::shared_ptr<GzClient> g_gz_client = NULL;

void signal_handler(int signum)
{
    (void)signum;
    g_stop = true;
}

void udp_link_entry_point()
{
    while (not g_stop) {
        g_udp_link->run_for(std::chrono::seconds(1));
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
        this->declare_parameter("port", 4243);
        this->declare_parameter("vehicle", "b3rb");

        std::string host = this->get_parameter("host").as_string();
        int port = this->get_parameter("port").as_int();
        std::string vehicle = this->get_parameter("vehicle").as_string();

        // create udp link
        g_udp_link = std::make_shared<UDPLink>(host, port);

        // create gz client
        g_gz_client = std::make_shared<GzClient>(vehicle);
        g_udp_link.get()->gz_ = g_gz_client;
        g_gz_client->udp_link_ = g_udp_link;

        // start threads
        udp_link_thread_ = std::make_shared<std::thread>(udp_link_entry_point);
        gz_thread_ = std::make_shared<std::thread>(gz_entry_point);
    }
    virtual ~SynapseGz()
    {
        // join threads
        g_stop = true;
        udp_link_thread_->join();
        gz_thread_->join();
    }

private:
    std::shared_ptr<std::thread> udp_link_thread_;
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
