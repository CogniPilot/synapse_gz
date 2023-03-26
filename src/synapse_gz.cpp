#include <atomic>
#include <chrono>
#include <csignal>
#include <gz/msgs/details/imu.pb.h>
#include <gz/msgs/details/imu_sensor.pb.h>
#include <gz/msgs/details/magnetometer.pb.h>
#include <gz/msgs/details/navsat.pb.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>


static std::atomic<bool> g_terminatePub(false);


class GzListener : public gz::transport::Node  {
  private:
    std::string topic_sub_altimeter_;
    std::string topic_sub_imu_;
    std::string topic_sub_magnetometer_;
    std::string topic_sub_navsat_;
    std::string topic_pub_actuators_;
  public:

    GzListener(std::string prefix) {
        topic_sub_altimeter_ = prefix + "altimeter_sensor/altimeter";
        topic_sub_imu_ = prefix + "imu_sensor/imu";
        topic_sub_magnetometer_ = prefix + "mag_sensor/magnetometer";
        topic_sub_navsat_ = prefix + "navsat_sensor/navsat";
        topic_pub_actuators_ = "/actuators";

        // altimeter sub
        boost::function<void (const gz::msgs::Altimeter&)> f_altimeter(
                boost::bind(&GzListener::handle_Altimeter, this, boost::placeholders::_1));
        if (!Subscribe<gz::msgs::Altimeter>(topic_sub_altimeter_, f_altimeter))
        {
            std::runtime_error("Error subscribing to topic " + topic_sub_altimeter_);
        }

        // imu sub
        boost::function<void (const gz::msgs::IMU&)> f_IMU(
                boost::bind(&GzListener::handle_IMU, this, boost::placeholders::_1));
        if (!Subscribe<gz::msgs::IMU>(topic_sub_imu_, f_IMU))
        {
            std::runtime_error("Error subscribing to topic " + topic_sub_imu_);
        }

        // magnetometer sub
        boost::function<void (const gz::msgs::Magnetometer&)> cb_mag(
                boost::bind(&GzListener::handle_Magnetometer, this, boost::placeholders::_1));
        if (!Subscribe<gz::msgs::Magnetometer>(topic_sub_magnetometer_, cb_mag))
        {
            std::runtime_error("Error subscribing to topic " + topic_sub_magnetometer_);
        }

        // navsat sub
        boost::function<void (const gz::msgs::NavSat&)> cb_navsat(
                boost::bind(&GzListener::handle_NavSat, this, boost::placeholders::_1));
        if (!Subscribe<gz::msgs::NavSat>(topic_sub_navsat_, cb_navsat))
        {
            std::runtime_error("Error subscribing to topic " + topic_sub_navsat_);
        }

        // actuators pub
        auto pub = Advertise<gz::msgs::Actuators>(topic_pub_actuators_);
        if (!pub)
        {
            std::runtime_error("Error advertising topic " + topic_pub_actuators_);
        }
    }

    void handle_Magnetometer(const gz::msgs::Magnetometer &_msg)
    {
        std::cout << "Magnetometer: " << _msg.field_tesla().x() << std::endl;
    }

    void handle_IMU(const gz::msgs::IMU &_msg)
    {
        std::cout << "IMU: " << _msg.angular_velocity().x() << std::endl;
    }

    void handle_NavSat(const gz::msgs::NavSat &_msg)
    {
        std::cout << "navsat: " << _msg.latitude_deg() << std::endl;
    }

    void handle_Altimeter(const gz::msgs::Altimeter &_msg)
    {
        std::cout << "altimeter: " << _msg.vertical_position() << std::endl;
    }
};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    std::string prefix = "/world/default/model/elm4/link/sensors/sensor/";
    GzListener listener(prefix);
    std::cout << "waiting for shutdown " << std::endl;
    gz::transport::waitForShutdown();
    return 0;
}

// vi: ts=4 sw=4 et
