#include "gz_client.hpp"
#include "udp_link.hpp"

#include <boost/bind/bind.hpp>
#include <boost/function.hpp>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <gz/msgs/details/logical_camera_image.pb.h>
#include <gz/msgs/details/material_color.pb.h>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/sim_clock.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

using namespace google::protobuf::util;

GzClient::GzClient(std::string vehicle)
{
    imu_audio_attack_ = false;
    vehicle_ = vehicle;
    model_prefix_ = "/model/" + vehicle;
    std::string world_prefix = "/world/default";
    std::string sensor_prefix = world_prefix + model_prefix_ + "/link/sensors/sensor";
    topic_sub_clock_ = "/clock";

    // sensors
    topic_sub_altimeter_ = sensor_prefix + "/altimeter_sensor/altimeter";
    topic_sub_imu_ = sensor_prefix + "/imu_sensor/imu";
    topic_sub_magnetometer_ = sensor_prefix + "/mag_sensor/magnetometer";
    topic_sub_navsat_ = sensor_prefix + "/navsat_sensor/navsat";
    topic_sub_wheel_odometry_ = world_prefix + model_prefix_ + "/joint_state";
    topic_sub_odometry_ = model_prefix_ + "/odometry";
    topic_sub_logical_camera_ = "/audio_source";

    // actuators
    topic_pub_actuators_ = "/actuators";
    topic_pub_lighting_config_ = world_prefix + "/light_config";
    topic_pub_material_color_ = world_prefix + "/material_color";

    // model prefix
    topic_sub_battery_state_ = model_prefix_ + "/battery/linear_battery/state";

    // clock sub
    boost::function<void(const gz::msgs::Clock&)> f_clock(
        boost::bind(&GzClient::handle_Clock, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Clock>(topic_sub_clock_, f_clock)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_clock_);
    } else {
        std::cout << "subscribed to " << topic_sub_clock_ << std::endl;
    }

    // altimeter sub
    boost::function<void(const gz::msgs::Altimeter&)> f_altimeter(
        boost::bind(&GzClient::handle_Altimeter, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Altimeter>(topic_sub_altimeter_, f_altimeter)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_altimeter_);
    } else {
        std::cout << "subscribed to " << topic_sub_altimeter_ << std::endl;
    }

    // imu sub
    boost::function<void(const gz::msgs::IMU&)> f_IMU(
        boost::bind(&GzClient::handle_IMU, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::IMU>(topic_sub_imu_, f_IMU)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_imu_);
    } else {
        std::cout << "subscribed to " << topic_sub_imu_ << std::endl;
    }

    // magnetometer sub
    boost::function<void(const gz::msgs::Magnetometer&)> cb_mag(
        boost::bind(&GzClient::handle_Magnetometer, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Magnetometer>(topic_sub_magnetometer_, cb_mag)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_magnetometer_);
    } else {
        std::cout << "subscribed to " << topic_sub_magnetometer_ << std::endl;
    }

    // navsat sub
    boost::function<void(const gz::msgs::NavSat&)> cb_navsat(
        boost::bind(&GzClient::handle_NavSat, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::NavSat>(topic_sub_navsat_, cb_navsat)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_navsat_);
    } else {
        std::cout << "subscribed to " << topic_sub_navsat_ << std::endl;
    }

    // wheel_odometry sub
    boost::function<void(const gz::msgs::Model&)> cb_wheel_odometry(
        boost::bind(&GzClient::handle_WheelOdometry, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Model>(topic_sub_wheel_odometry_, cb_wheel_odometry)) {
        throw std::runtime_error("Error subscribing to wheel odometry " + topic_sub_wheel_odometry_);
    } else {
        std::cout << "subscribed to " << topic_sub_wheel_odometry_ << std::endl;
    }

    // odometry sub
    boost::function<void(const gz::msgs::Odometry&)> cb_odometry(
        boost::bind(&GzClient::handle_Odometry, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::Odometry>(topic_sub_odometry_, cb_odometry)) {
        throw std::runtime_error("Error subscribing to wheel odometry " + topic_sub_odometry_);
    } else {
        std::cout << "subscribed to " << topic_sub_odometry_ << std::endl;
    }

    // battery sub
    boost::function<void(const gz::msgs::BatteryState&)> cb_battery_state(
        boost::bind(&GzClient::handle_BatteryState, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::BatteryState>(topic_sub_battery_state_, cb_battery_state)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_battery_state_);
    } else {
        std::cout << "subscribed to " << topic_sub_battery_state_ << std::endl;
    }

    // logical camera sub
    boost::function<void(const gz::msgs::LogicalCameraImage&)> cb_logical_camera(
        boost::bind(&GzClient::handle_LogicalCamera, this, boost::placeholders::_1));
    if (!Subscribe<gz::msgs::LogicalCameraImage>(topic_sub_logical_camera_, cb_logical_camera)) {
        throw std::runtime_error("Error subscribing to topic " + topic_sub_logical_camera_);
    } else {
        std::cout << "subscribed to " << topic_sub_logical_camera_ << std::endl;
    }

    // actuators pub
    pub_actuators_ = Advertise<gz::msgs::Actuators>(topic_pub_actuators_);
    if (!pub_actuators_) {
        throw std::runtime_error("Error advertising topic " + topic_pub_actuators_);
    } else {
        std::cout << "publishing to " << topic_pub_actuators_ << std::endl;
    }

    // lighting config pub
    pub_lighting_config_ = Advertise<gz::msgs::Light>(topic_pub_lighting_config_);
    if (!pub_actuators_) {
        throw std::runtime_error("Error advertising topic " + topic_pub_lighting_config_);
    } else {
        std::cout << "publishing to " << topic_pub_lighting_config_ << std::endl;
    }

    // material color pub
    pub_material_color_ = Advertise<gz::msgs::MaterialColor>(topic_pub_material_color_);
    if (!pub_actuators_) {
        throw std::runtime_error("Error advertising topic " + topic_pub_material_color_);
    } else {
        std::cout << "publishing to " << topic_pub_material_color_ << std::endl;
    }
}

void GzClient::handle_Clock(const gz::msgs::Clock& msg)
{
    synapse_pb::SimClock syn_msg;
    if (msg.has_sim()) {
        syn_msg.mutable_sim()->set_sec(msg.sim().sec());
        syn_msg.mutable_sim()->set_nanosec(msg.sim().nsec());
    }
    if (msg.has_real()) {
        syn_msg.mutable_real()->set_sec(msg.real().sec());
        syn_msg.mutable_real()->set_nanosec(msg.real().nsec());
    }
    if (msg.has_system()) {
        syn_msg.mutable_system()->set_sec(msg.system().sec());
        syn_msg.mutable_system()->set_nanosec(msg.system().nsec());
    }
    if (msg.has_header()) {
        if (msg.header().has_stamp()) {
            syn_msg.mutable_header()->mutable_stamp()->set_sec(msg.header().stamp().sec());
            syn_msg.mutable_header()->mutable_stamp()->set_nanosec(msg.header().stamp().nsec());
        }
    }

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_SIM_CLOCK);
    frame.set_allocated_sim_clock(&syn_msg);
    udp_send(frame);
    frame.release_sim_clock();
}

void GzClient::handle_Magnetometer(const gz::msgs::Magnetometer& msg)
{
    // construct message
    synapse_pb::MagneticField syn_msg;
    syn_msg.mutable_magnetic_field()->set_x(msg.field_tesla().x());
    syn_msg.mutable_magnetic_field()->set_y(msg.field_tesla().y());
    syn_msg.mutable_magnetic_field()->set_z(msg.field_tesla().z());

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_MAGNETIC_FIELD);
    frame.set_allocated_magnetic_field(&syn_msg);
    udp_send(frame);
    frame.release_magnetic_field();
}

void GzClient::handle_IMU(const gz::msgs::IMU& msg)
{
    int64_t sec = msg.header().stamp().sec();
    int64_t nsec = msg.header().stamp().nsec();
    double t = sec + nsec * 1e-9;
    double A = 80 * M_PI / 180.0;
    double attack = 0;
    double f = 1.5;
    if (imu_audio_attack_) {
        // printf("imu attack!\n");
        if (sin(2 * M_PI * f * t) > 0) {
            attack = A;
        } else {
            attack = -A;
        }
    } else {
        // printf("no imu attack!\n");
    }

    // construct message
    synapse_pb::Imu syn_msg;
    syn_msg.mutable_linear_acceleration()->set_x(msg.linear_acceleration().x());
    syn_msg.mutable_linear_acceleration()->set_y(msg.linear_acceleration().y());
    syn_msg.mutable_linear_acceleration()->set_z(msg.linear_acceleration().z());
    syn_msg.mutable_angular_velocity()->set_x(msg.angular_velocity().x() + attack);
    syn_msg.mutable_angular_velocity()->set_y(msg.angular_velocity().y());
    syn_msg.mutable_angular_velocity()->set_z(msg.angular_velocity().z());

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_IMU);
    frame.set_allocated_imu(&syn_msg);
    udp_send(frame);
    frame.release_imu();
}

void GzClient::handle_NavSat(const gz::msgs::NavSat& msg)
{
    // construct message
    synapse_pb::NavSatFix syn_msg;
    syn_msg.mutable_header()->set_frame_id(msg.frame_id());
    syn_msg.mutable_header()->mutable_stamp()->set_sec(
        msg.header().stamp().sec());
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(
        msg.header().stamp().nsec());
    syn_msg.set_latitude(msg.latitude_deg());
    syn_msg.set_longitude(msg.longitude_deg());
    syn_msg.set_altitude(msg.altitude());

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_NAV_SAT_FIX);
    frame.set_allocated_nav_sat_fix(&syn_msg);
    udp_send(frame);
    frame.release_nav_sat_fix();
}

void GzClient::handle_Altimeter(const gz::msgs::Altimeter& msg)
{
    synapse_pb::Altimeter syn_msg;
    syn_msg.set_vertical_position(msg.vertical_position());
    syn_msg.set_vertical_velocity(msg.vertical_velocity());
    syn_msg.set_vertical_reference(msg.vertical_reference());

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_ALTIMETER);
    frame.set_allocated_alitimeter(&syn_msg);
    udp_send(frame);
    frame.release_alitimeter();
}

void GzClient::handle_BatteryState(const gz::msgs::BatteryState& msg)
{
    // construct message
    static std::map<gz::msgs::BatteryState::PowerSupplyStatus, synapse_pb::BatteryState::PowerSupplyStatus> power_supply_status_map = {
        { gz::msgs::BatteryState_PowerSupplyStatus_UNKNOWN, synapse_pb::BatteryState_PowerSupplyStatus_UNKNOWN_STATUS },
        { gz::msgs::BatteryState_PowerSupplyStatus_CHARGING, synapse_pb::BatteryState_PowerSupplyStatus_CHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_DISCHARGING, synapse_pb::BatteryState_PowerSupplyStatus_DISCHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_NOT_CHARGING, synapse_pb::BatteryState_PowerSupplyStatus_NOT_CHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_FULL, synapse_pb::BatteryState_PowerSupplyStatus_FULL }
    };
    synapse_pb::BatteryState syn_msg;
    syn_msg.set_voltage(msg.voltage());
    syn_msg.set_current(msg.current());
    syn_msg.set_charge(msg.charge());
    syn_msg.set_capacity(msg.capacity());
    syn_msg.set_percentage(msg.percentage());
    syn_msg.set_power_supply_status(power_supply_status_map[msg.power_supply_status()]);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_BATTERY_STATE);
    frame.set_allocated_battery_state(&syn_msg);
    udp_send(frame);
    frame.release_battery_state();
}

void GzClient::handle_LogicalCamera(const gz::msgs::LogicalCameraImage& msg)
{
    bool detected = false;
    for (int i = 0; i < msg.model_size(); i++) {
        if (msg.model(i).name() == vehicle_) {
            detected = true;
        }
    }
    imu_audio_attack_ = detected;
}

void GzClient::handle_WheelOdometry(const gz::msgs::Model& msg)
{
    // construct message
    synapse_pb::WheelOdometry syn_msg;
    // assume differential so average all wheels for odom
    // where odom is mounted on motor
    int n_wheels = msg.joint_size();
    double rotation = 0;
    for (int i = 0; i < n_wheels; i++) {
        rotation += msg.joint(i).axis1().position();
    }
    rotation /= n_wheels;
    syn_msg.set_rotation(rotation);

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_WHEEL_ODOMETRY);
    frame.set_allocated_wheel_odometry(&syn_msg);
    udp_send(frame);
    frame.release_wheel_odometry();
}

void GzClient::handle_Odometry(const gz::msgs::Odometry& msg)
{
    // construct message
    synapse_pb::Odometry syn_msg;

    if (msg.has_header()) {
        for (auto map = msg.header().data().begin(); map < msg.header().data().end(); map++) {
            if (map->key() == "frame_id") {
                syn_msg.mutable_header()->set_frame_id(map->value(0));
            } else if (map->key() == "child_frame_id") {
                syn_msg.set_child_frame_id(map->value(0));
            }
        }
        if (msg.header().has_stamp()) {
            syn_msg.mutable_header()->mutable_stamp()->set_sec(
                msg.header().stamp().sec());
            syn_msg.mutable_header()->mutable_stamp()->set_nanosec(
                msg.header().stamp().nsec());
        }
    }

    if (msg.has_pose()) {
        if (msg.pose().has_position()) {
            syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_x(msg.pose().position().x());
            syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_y(msg.pose().position().y());
            syn_msg.mutable_pose()->mutable_pose()->mutable_position()->set_z(msg.pose().position().z());
        }
        if (msg.pose().has_orientation()) {
            syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(msg.pose().orientation().x());
            syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(msg.pose().orientation().y());
            syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(msg.pose().orientation().z());
            syn_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(msg.pose().orientation().w());
        }
    }

    if (msg.has_twist()) {
        if (msg.twist().has_linear()) {
            syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_x(msg.twist().linear().x());
            syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_y(msg.twist().linear().y());
            syn_msg.mutable_twist()->mutable_twist()->mutable_linear()->set_z(msg.twist().linear().z());
        }
        if (msg.twist().has_angular()) {
            syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_x(msg.twist().angular().x());
            syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_y(msg.twist().angular().y());
            syn_msg.mutable_twist()->mutable_twist()->mutable_angular()->set_z(msg.twist().angular().z());
        }
    }

    // serialize message
    synapse_pb::Frame frame {};
    frame.set_topic(synapse_pb::Topic::TOPIC_ODOMETRY);
    frame.set_allocated_odometry(&syn_msg);
    udp_send(frame);
    frame.release_odometry();
}

void GzClient::publish_actuators(const synapse_pb::Actuators& msg)
{
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

    pub_actuators_.Publish(gz_msg);
}

void GzClient::publish_led_array(const synapse_pb::LEDArray& msg)
{
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
            pub_material_color_.Publish(msg);
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

            pub_lighting_config_.Publish(msg);
        }
    }
}

void GzClient::udp_send(const synapse_pb::Frame& frame) const
{
    std::stringstream stream;
    if (!SerializeDelimitedToOstream(frame, &stream)) {
        std::cerr << "Failed to serialize " << frame.topic() << std::endl;
        return;
    }
    if (udp_link_ != nullptr) {
        udp_link_.get()->write((const uint8_t*)stream.str().c_str(), stream.str().length());
    }
}

// vi: ts=4 sw=4 et
