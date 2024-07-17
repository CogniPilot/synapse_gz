#ifndef GZ_CLIENT_HPP
#define GZ_CLIENT_HPP

#include <gz/msgs/details/logical_camera_image.pb.h>
#include <gz/transport/Publisher.hh>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include "synapse_pb/led_array.pb.h"
#include <synapse_pb/actuators.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/twist.pb.h>

class UDPLink;

class GzClient : public gz::transport::Node {
private:
    bool imu_audio_attack_;
    std::string vehicle_;
    std::string model_prefix_;
    std::string topic_sub_clock_;
    std::string topic_sub_altimeter_;
    std::string topic_sub_imu_;
    std::string topic_sub_magnetometer_;
    std::string topic_sub_navsat_;
    std::string topic_pub_actuators_;
    std::string topic_pub_lighting_config_;
    std::string topic_pub_material_color_;
    std::string topic_sub_battery_state_;
    std::string topic_sub_wheel_odometry_;
    std::string topic_sub_odometry_;
    std::string topic_sub_logical_camera_;

public:
    std::shared_ptr<UDPLink> udp_link_ { NULL };
    gz::transport::Node::Publisher pub_actuators_;
    gz::transport::Node::Publisher pub_lighting_config_;
    gz::transport::Node::Publisher pub_material_color_;
    GzClient(std::string prefix);

    // gazebo listeners
    void handle_Clock(const gz::msgs::Clock& msg);
    void handle_Magnetometer(const gz::msgs::Magnetometer& msg);
    void handle_IMU(const gz::msgs::IMU& msg);
    void handle_NavSat(const gz::msgs::NavSat& msg);
    void handle_Altimeter(const gz::msgs::Altimeter& msg);
    void handle_BatteryState(const gz::msgs::BatteryState& msg);
    void handle_WheelOdometry(const gz::msgs::Model& msg);
    void handle_Odometry(const gz::msgs::Odometry& msg);
    void handle_LogicalCamera(const gz::msgs::LogicalCameraImage& msg);

    // publish to gazebo
    void publish_actuators(const synapse_pb::Actuators& msg);
    void publish_led_array(const synapse_pb::LEDArray& msg);

    // write
    void udp_send(const synapse_pb::Frame& frame) const;
};

// vi: ts=4 sw=4 et

#endif // GZ_CLIENT_HPP
