#include <atomic>
#include <boost/asio/connect.hpp>
#include <boost/asio/write.hpp>
#include <gz/transport/Node.hh>
#include <string>
#include <chrono>
#include <csignal>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>

#include <thread>
#include <mutex>
#include <atomic>

// tcp
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_at.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <boost/asio.hpp>

// synapse
#include "synapse_protobuf/actuators.pb.h"
#include "synapse_protobuf/twist.pb.h"
#include "synapse_protobuf/joy.pb.h"
#include "synapse_protobuf/odometry.pb.h"
#include "synapse_tinyframe/TinyFrame.h"
#include "synapse_tinyframe/utils.h"
#include "synapse_tinyframe/SynapseTopics.h"

std::atomic<bool> stop{false};

using boost::asio::ip::tcp;
static const uint32_t rx_buf_length = 1024;
std::mutex guard_rx_buf;
static uint8_t rx_buf[rx_buf_length];

static std::shared_ptr<boost::asio::deadline_timer> timer;
static boost::asio::io_context io_context;
static tcp::socket sockfd = tcp::socket(io_context);
static tcp::resolver resolver(io_context);
static TinyFrame * tf0;

void tx_handler(const boost::system::error_code & error, std::size_t bytes_transferred) {
    (void) bytes_transferred;
    if (error.failed()) {
        std::cerr << error.message() << std::endl;
    }
}

void rx_handler(const boost::system::error_code & error, std::size_t bytes_transferred) {
    if (error.failed()) {
        std::cerr << error.message() << std::endl;
    }
    const std::lock_guard<std::mutex> lock(guard_rx_buf);
    TF_Accept(tf0, rx_buf, bytes_transferred);
}

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
    (void)tf;
    boost::asio::async_write(sockfd, boost::asio::buffer(buf, len), tx_handler);
}

TF_Result actuatorsListener(TinyFrame *tf, TF_Msg *frame)
{
    (void)tf;
    Actuators msg;
    std::string data((char *)frame->data, frame->len);
    if (!msg.ParseFromString(data)) {
        std::cerr << "Failed to parse actuators" << std::endl;
        return TF_STAY;
    }
    return TF_STAY;
}

TF_Result out_cmd_vel_Listener(TinyFrame *tf, TF_Msg *frame)
{
    (void)tf;
    Twist msg;
    std::string data((char *)frame->data, frame->len);
    if (!msg.ParseFromString(data)) {
        std::cerr << "Failed to out_cmd_vel" << std::endl;
        return TF_STAY;
    } else {
        std::cout << "out cmd vel"
            << msg.linear().x()
            << msg.linear().y()
            << msg.linear().z()
            << msg.angular().x()
            << msg.angular().y()
            << msg.angular().z()
            << std::endl;
    }
    return TF_STAY;
}

TF_Result genericListener(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    dumpFrameInfo(msg);
    return TF_STAY;
}

void tick(const boost::system::error_code& /*e*/) {
    sockfd.async_read_some(boost::asio::buffer(rx_buf, rx_buf_length), rx_handler);
    if (not stop) {
        timer->expires_at(timer->expires_at() + boost::posix_time::milliseconds(100));
        timer->async_wait(tick);
    }
}

class GzListener : public gz::transport::Node  {
  private:
    std::string topic_sub_clock_;
    std::string topic_sub_altimeter_;
    std::string topic_sub_imu_;
    std::string topic_sub_magnetometer_;
    std::string topic_sub_navsat_;
    std::string topic_pub_actuators_;
  public:

    GzListener(std::string prefix) {
        topic_sub_clock_ = "/clock";
        topic_sub_altimeter_ = prefix + "altimeter_sensor/altimeter";
        topic_sub_imu_ = prefix + "imu_sensor/imu";
        topic_sub_magnetometer_ = prefix + "mag_sensor/magnetometer";
        topic_sub_navsat_ = prefix + "navsat_sensor/navsat";
        topic_pub_actuators_ = "/actuators";

        // clock sub
        boost::function<void (const gz::msgs::Clock&)> f_clock(
                boost::bind(&GzListener::handle_Clock, this, boost::placeholders::_1));
        if (!Subscribe<gz::msgs::Clock>(topic_sub_clock_, f_clock))
        {
            std::runtime_error("Error subscribing to topic " + topic_sub_clock_);
        }

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

    void handle_Clock(const gz::msgs::Clock &msg)
    {
        TF_Msg frame;
        frame.type = SYNAPSE_SIM_CLOCK_TOPIC;
        std::string data;
        if (!msg.SerializeToString(&data)) {
            std::cerr << "Failed to serialize Clock" << std::endl;
        }
        frame.len = data.length();
        frame.data = (const uint8_t *)data.c_str();
        TF_Send(tf0, &frame);
    }

    void handle_Magnetometer(const gz::msgs::Magnetometer &msg)
    {
        TF_Msg frame;
        frame.type = SYNAPSE_IN_MAG_TOPIC;
        std::string data;
        if (!msg.SerializeToString(&data)) {
            std::cerr << "Failed to serialize Magnetometer" << std::endl;
        }
        frame.len = data.length();
        frame.data = (const uint8_t *)data.c_str();
        //TF_Send(tf0, &frame);
    }

    void handle_IMU(const gz::msgs::IMU &msg)
    {
        TF_Msg frame;
        frame.type = SYNAPSE_IN_IMU_TOPIC;
        std::string data;
        if (!msg.SerializeToString(&data)) {
            std::cerr << "Failed to serialize IMU" << std::endl;
        }
        frame.len = data.length();
        frame.data = (const uint8_t *)data.c_str();
        //TF_Send(tf0, &frame);
    }

    void handle_NavSat(const gz::msgs::NavSat &msg)
    {
        TF_Msg frame;
        frame.type = SYNAPSE_IN_NAVSAT_TOPIC;
        std::string data;
        if (!msg.SerializeToString(&data)) {
            std::cerr << "Failed to serialize NavSat" << std::endl;
        }
        frame.len = data.length();
        frame.data = (const uint8_t *)data.c_str();
        //TF_Send(tf0, &frame);
    }

    void handle_Altimeter(const gz::msgs::Altimeter &msg)
    {
        TF_Msg frame;
        frame.type = SYNAPSE_IN_ALT_TOPIC;
        std::string data;
        if (!msg.SerializeToString(&data)) {
            std::cerr << "Failed to serialize IMU" << std::endl;
        }
        frame.len = data.length();
        frame.data = (const uint8_t *)data.c_str();
        //TF_Send(tf0, &frame);
    }
};


void gz_entry_point()
{
    std::cout << "gz thread started" << std::endl;
    std::string prefix = "/world/default/model/elm4/link/sensors/sensor/";
    GzListener listener(prefix);
    while (not stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void signal_handler(
    const boost::system::error_code& error, int signal_number) {
    (void)error;
    (void)signal_number;
    stop = true;
}

void handle_connect(
    const boost::system::error_code& ec,
    const boost::asio::ip::tcp::endpoint& endpoint)
{
    if (ec.failed()) {
        std::cout << ec.message() << std::endl;
    } else {
        std::cout << "tcp connnected: " << endpoint << std::endl;
        timer->async_wait(tick);
    }
}

void tcp_entry_point(std::string host, int port)
{
    // Set up the TinyFrame library
    tf0 = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    tf0->usertag = 0;

    //TF_AddGenericListener(tf0, genericListener);
    TF_AddTypeListener(tf0, SYNAPSE_OUT_CMD_VEL_TOPIC, out_cmd_vel_Listener);
    TF_AddTypeListener(tf0, SYNAPSE_OUT_ACTUATORS_TOPIC, actuatorsListener);

    boost::asio::signal_set signals(io_context, SIGINT, SIGTERM);

    timer = std::make_shared<boost::asio::deadline_timer>(
            io_context, boost::posix_time::seconds(1));

    std::cout << "tcp connecting..." << std::endl;
    signals.async_wait(signal_handler);
    boost::asio::async_connect(
        sockfd,
        resolver.resolve(host, std::to_string(port)),
        handle_connect);
    while (not stop) {
        io_context.run_for(std::chrono::seconds(1));
    }
}

int main(int argc, char ** argv) {
    (void) argc;
    if (argc < 3) {
        std::cerr << argv[0] << "host port" << std::endl;
        return -1;
    }
    std::thread tcp_thread(
        tcp_entry_point, argv[1], std::atoi(argv[2]));
    std::thread gz_thread(gz_entry_point);
    tcp_thread.join();
    gz_thread.join();
    return 0;
}

// vi: ts=4 sw=4 et
