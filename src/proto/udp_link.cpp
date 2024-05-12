#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/utils.h>

#include "synapse_protobuf/led_array.pb.h"
#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/twist.pb.h>

#include <boost/asio/error.hpp>
#include <boost/system/error_code.hpp>

#include "../clients/gz_client.hpp"
#include "udp_link.hpp"

using boost::asio::ip::udp;
using std::placeholders::_1;
using std::placeholders::_2;

static void write_udp(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    // get udp link attached to tf pointer in userdata
    UDPLink* udp_link = (UDPLink*)tf->userdata;

    // write buffer to udp link
    udp_link->write(buf, len);
}

UDPLink::UDPLink(std::string host, int port)
{
    remote_endpoint_ = *udp::resolver(io_context_).resolve(udp::resolver::query(host, std::to_string(port)));
    my_endpoint_ = udp::endpoint(udp::v4(), GZ_PORT);

    // Set up the TinyFrame library
    tf_ = std::make_shared<TinyFrame>(*TF_Init(TF_MASTER, write_udp));
    tf_->usertag = 0;
    tf_->userdata = this;
    tf_->write = write_udp;
    TF_AddGenericListener(tf_.get(), UDPLink::generic_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_CMD_VEL_TOPIC, UDPLink::out_cmd_vel_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_ACTUATORS_TOPIC, UDPLink::actuators_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_ODOMETRY_TOPIC, UDPLink::odometry_listener);
    TF_AddTypeListener(tf_.get(), SYNAPSE_LED_ARRAY_TOPIC, UDPLink::led_array_listener);

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

void UDPLink::tx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "tx error: " << ec.message() << std::endl;
    }
}

void UDPLink::rx_handler(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
    if (ec == boost::asio::error::eof) {
        std::cerr << "reconnecting due to eof" << std::endl;
    } else if (ec == boost::asio::error::connection_reset) {
        std::cerr << "reconnecting due to reset" << std::endl;
    } else if (ec != boost::system::errc::success) {
        std::cerr << "rx error: " << ec.message() << std::endl;
    } else if (ec == boost::system::errc::success) {
        const std::lock_guard<std::mutex> lock(guard_rx_buf_);
        TF_Accept(tf_.get(), rx_buf_, bytes_transferred);
    }

    // schedule new rx
    sock_.async_receive_from(boost::asio::buffer(rx_buf_, rx_buf_length_),
        my_endpoint_,
        std::bind(&UDPLink::rx_handler, this, _1, _2));
}

TF_Result UDPLink::actuators_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse::msgs::Actuators msg;

    // get udp client attached to tf pointer in userdata
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    GzClient* gz_client = udp_link->gz_.get();

    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse actuators" << std::endl;
        return TF_STAY;
    }
    gz::msgs::Actuators gz_msg;

    for (auto it = msg.position().begin(); it != msg.position().end(); ++it) {
        gz_msg.add_position(*it);
    }

    for (auto it = msg.velocity().begin(); it != msg.velocity().end(); ++it) {
        gz_msg.add_velocity(*it);
    }

    for (auto it = msg.normalized().begin(); it != msg.normalized().end(); ++it) {
        gz_msg.add_normalized(*it);
    }

    gz_client->pub_actuators_.Publish(gz_msg);
    return TF_STAY;
}

TF_Result UDPLink::out_cmd_vel_listener(TinyFrame* tf, TF_Msg* frame)
{
    (void)tf;
    synapse::msgs::Twist msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse out_cmd_vel" << std::endl;
        return TF_STAY;
    } else {
    }
    return TF_STAY;
}

TF_Result UDPLink::odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    (void)tf;
    synapse::msgs::Odometry msg;
    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse odometry" << std::endl;
        return TF_STAY;
    } else {
    }
    return TF_STAY;
}

TF_Result UDPLink::led_array_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse::msgs::LEDArray msg;

    // get udp client attached to tf pointer in userdata
    UDPLink* udp_link = (UDPLink*)tf->userdata;
    GzClient* gz_client = udp_link->gz_.get();

    if (!msg.ParseFromArray(frame->data, frame->len)) {
        std::cerr << "Failed to parse led array" << std::endl;
        return TF_STAY;
    }

    for (int i = 0; i < msg.led_size(); ++i) {
        auto led = msg.led(i);

        std::string name = "led_" + std::to_string(led.index());

        double intensity = 5.0 * (led.r() + led.b() + led.g()) / (3 * 255.0);
        double scale = 10.0;

        {
            gz::msgs::MaterialColor msg;
            msg.set_entity_match(gz::msgs::MaterialColor_EntityMatch::MaterialColor_EntityMatch_ALL);
            msg.mutable_entity()->set_name(name);
            msg.mutable_ambient()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_ambient()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_ambient()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_ambient()->set_a(0.75);

            msg.mutable_diffuse()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_diffuse()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_diffuse()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_diffuse()->set_a(0.75);

            msg.mutable_specular()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_specular()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_specular()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_specular()->set_a(0.75);

            msg.mutable_emissive()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_emissive()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_emissive()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_emissive()->set_a(0.75);
            gz_client->pub_material_color_.Publish(msg);
        }

        {
            gz::msgs::Light msg;
            msg.set_name(name);
            msg.set_type(gz::msgs::Light::LightType::Light_LightType_SPOT);

            msg.mutable_diffuse()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_diffuse()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_diffuse()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_diffuse()->set_a(0.75);

            msg.mutable_specular()->set_r(std::min(scale * led.r() / 255.0, 1.0));
            msg.mutable_specular()->set_g(std::min(scale * led.g() / 255.0, 1.0));
            msg.mutable_specular()->set_b(std::min(scale * led.b() / 255.0, 1.0));
            msg.mutable_specular()->set_a(0.75);

            msg.set_spot_inner_angle(0.5);
            msg.set_spot_outer_angle(1.5);
            msg.set_spot_falloff(0.0);
            msg.set_attenuation_constant(1.0);
            msg.set_attenuation_linear(1.0);
            msg.set_attenuation_quadratic(1.0);
            msg.set_intensity(intensity);
            msg.set_range(100.0);

            gz_client->pub_lighting_config_.Publish(msg);
        }
    }

    return TF_STAY;
}

TF_Result UDPLink::generic_listener(TinyFrame* tf, TF_Msg* msg)
{
    (void)tf;
    int type = msg->type;
    std::cout << "generic listener id:" << type << std::endl;
    dumpFrameInfo(msg);
    return TF_STAY;
}

void UDPLink::run_for(std::chrono::seconds sec)
{
    io_context_.run_for(std::chrono::seconds(sec));
}

void UDPLink::write(const uint8_t* buf, uint32_t len)
{
    sock_.async_send_to(boost::asio::buffer(buf, len),
        remote_endpoint_,
        std::bind(&UDPLink::tx_handler, this, _1, _2));
}

// vi: ts=4 sw=4 et
