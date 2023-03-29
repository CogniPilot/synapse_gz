#ifndef GZ_CLIENT_HPP
#define GZ_CLIENT_HPP

#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include "synapse_tinyframe/TinyFrame.h"
#include "tcp_client.hpp"


class GzClient : public gz::transport::Node  {
  private:
    std::string topic_sub_clock_;
    std::string topic_sub_altimeter_;
    std::string topic_sub_imu_;
    std::string topic_sub_magnetometer_;
    std::string topic_sub_navsat_;
    std::string topic_pub_actuators_;
    std::shared_ptr<TinyFrame> tf_;
  public:
    GzClient(std::string prefix, std::shared_ptr<TinyFrame> const & tf);
    void tf_send(TF_Msg & frame);
    void handle_Clock(const gz::msgs::Clock &msg);
    void handle_Magnetometer(const gz::msgs::Magnetometer &msg);
    void handle_IMU(const gz::msgs::IMU &msg);
    void handle_NavSat(const gz::msgs::NavSat &msg);
    void handle_Altimeter(const gz::msgs::Altimeter &msg);
};

// vi: ts=4 sw=4 et

#endif // GZ_CLIENT_HPP