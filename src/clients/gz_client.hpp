#ifndef GZ_CLIENT_HPP
#define GZ_CLIENT_HPP

#include <gz/transport/Publisher.hh>
#include <memory>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include "synapse_tinyframe/TinyFrame.h"

class TcpClient;

class GzClient : public gz::transport::Node {
private:
    std::string topic_sub_clock_;
    std::string topic_sub_altimeter_;
    std::string topic_sub_imu_;
    std::string topic_sub_magnetometer_;
    std::string topic_sub_navsat_;
    std::string topic_pub_actuators_;
    std::string topic_sub_battery_state_;
    std::shared_ptr<TinyFrame> tf_;

public:
    TcpClient * tcp_client_;
    gz::transport::Node::Publisher pub_actuators_;
    GzClient(std::string prefix, std::shared_ptr<TinyFrame> const& tf);
    void tf_send(TF_Msg& frame);
    void handle_Clock(const gz::msgs::Clock& msg);
    void handle_Magnetometer(const gz::msgs::Magnetometer& msg);
    void handle_IMU(const gz::msgs::IMU& msg);
    void handle_NavSat(const gz::msgs::NavSat& msg);
    void handle_Altimeter(const gz::msgs::Altimeter& msg);
    void handle_BatteryState(const gz::msgs::BatteryState& msg);
};

// vi: ts=4 sw=4 et

#endif // GZ_CLIENT_HPP
