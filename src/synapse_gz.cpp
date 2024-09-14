#include "rclcpp/rclcpp.hpp"

#include "link/gz_client.hpp"
#include "link/udp_link.hpp"

#include <memory>
#include <rclcpp/parameter_value.hpp>

using namespace std::chrono_literals;

class SynapseGz : public rclcpp::Node {
public:
    SynapseGz();
    virtual ~SynapseGz();

private:
    void udp_run();
    std::shared_ptr<std::thread> udp_link_thread_;
    std::shared_ptr<UDPLink> udp_link_ = NULL;
    std::shared_ptr<GzClient> gz_client_ = NULL;
};

void SynapseGz::udp_run()
{
    while (rclcpp::ok()) {
        udp_link_->run_for(1s);
    }
}

SynapseGz::SynapseGz()
    : Node("synapze_gz")
{
    this->declare_parameter("host", "127.0.0.1");
    this->declare_parameter("port", 4243);
    this->declare_parameter("vehicle", "b3rb");

    std::string host = this->get_parameter("host").as_string();
    int port = this->get_parameter("port").as_int();
    std::string vehicle = this->get_parameter("vehicle").as_string();

    gz_client_ = std::make_shared<GzClient>(vehicle);
    udp_link_ = std::make_shared<UDPLink>(host, port, gz_client_.get());

    // start threads
    udp_link_thread_ = std::make_shared<std::thread>(std::bind(
        &SynapseGz::udp_run, this));
}

SynapseGz::~SynapseGz()
{
    // join threads
    udp_link_thread_->join();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynapseGz>());
    rclcpp::shutdown();

    return 0;
}

// vi: ts=4 sw=4 et
