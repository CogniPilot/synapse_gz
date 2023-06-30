#include "gz_client.hpp"

#include <boost/bind/bind.hpp>
#include <boost/function.hpp>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>

GzClient::GzClient(std::string prefix, std::shared_ptr<TinyFrame> const& tf)
    : tf_(tf)
{
    topic_sub_clock_ = "/clock";
    topic_sub_altimeter_ = prefix + "altimeter_sensor/altimeter";
    topic_sub_imu_ = prefix + "imu_sensor/imu";
    topic_sub_magnetometer_ = prefix + "mag_sensor/magnetometer";
    topic_sub_navsat_ = prefix + "navsat_sensor/navsat";
    topic_pub_actuators_ = "/actuators";

    // clock sub
    boost::function<void(const gz::msgs::Clock&)> f_clock(
        boost::bind(&GzClient::handle_Clock, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Clock>(topic_sub_clock_, f_clock)) {
        std::runtime_error("Error subscribing to topic " + topic_sub_clock_);
    }

    // altimeter sub
    //boost::function<void(const gz::msgs::Altimeter&)> f_altimeter(
        //boost::bind(&GzClient::handle_Altimeter, this, boost::placeholders::_1));
    //if (!Subscribe<gz::msgs::Altimeter>(topic_sub_altimeter_, f_altimeter)) {
        //std::runtime_error("Error subscribing to topic " + topic_sub_altimeter_);
    //}

    // imu sub
    //boost::function<void(const gz::msgs::IMU&)> f_IMU(
        //boost::bind(&GzClient::handle_IMU, this, boost::placeholders::_1));
    //if (!Subscribe<gz::msgs::IMU>(topic_sub_imu_, f_IMU)) {
        //std::runtime_error("Error subscribing to topic " + topic_sub_imu_);
    //}

    // magnetometer sub
    //boost::function<void(const gz::msgs::Magnetometer&)> cb_mag(
        //boost::bind(&GzClient::handle_Magnetometer, this, boost::placeholders::_1));
    //if (!Subscribe<gz::msgs::Magnetometer>(topic_sub_magnetometer_, cb_mag)) {
        //std::runtime_error("Error subscribing to topic " + topic_sub_magnetometer_);
    //}

    // navsat sub
    boost::function<void(const gz::msgs::NavSat&)> cb_navsat(
        boost::bind(&GzClient::handle_NavSat, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::NavSat>(topic_sub_navsat_, cb_navsat)) {
        std::runtime_error("Error subscribing to topic " + topic_sub_navsat_);
    }

    // actuators pub
    pub_actuators_ = Advertise<gz::msgs::Actuators>(topic_pub_actuators_);
    if (!pub_actuators_) {
        std::runtime_error("Error advertising topic " + topic_pub_actuators_);
    }
}

void GzClient::tf_send(TF_Msg& frame)
{
    TF_Send(tf_.get(), &frame);
}

void GzClient::handle_Clock(const gz::msgs::Clock& msg)
{
    TF_Msg frame;
    frame.type = SYNAPSE_SIM_CLOCK_TOPIC;
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Clock" << std::endl;
        return;
    }
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_Magnetometer(const gz::msgs::Magnetometer& msg)
{
    TF_Msg frame;
    frame.type = SYNAPSE_IN_MAG_TOPIC;
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Magnetometer" << std::endl;
        return;
    }
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_IMU(const gz::msgs::IMU& msg)
{
    TF_Msg frame;
    frame.type = SYNAPSE_IN_IMU_TOPIC;
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize IMU" << std::endl;
        return;
    }
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_NavSat(const gz::msgs::NavSat& msg)
{
    TF_Msg frame;
    frame.type = SYNAPSE_IN_NAVSAT_TOPIC;
    synapse::msgs::NavSatFix syn_msg;

    syn_msg.mutable_header()->set_frame_id(msg.frame_id());
    syn_msg.mutable_header()->mutable_stamp()->set_seconds(
            msg.header().stamp().sec());
    syn_msg.mutable_header()->mutable_stamp()->set_nanos(
            msg.header().stamp().nsec());
    syn_msg.set_latitude(msg.latitude_deg());
    syn_msg.set_longitude(msg.longitude_deg());
    syn_msg.set_altitude(msg.altitude());

    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize NavSat" << std::endl;
        return;
    }
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_Altimeter(const gz::msgs::Altimeter& msg)
{
    TF_Msg frame;
    frame.type = SYNAPSE_IN_ALT_TOPIC;
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize IMU" << std::endl;
        return;
    }
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

// vi: ts=4 sw=4 et
