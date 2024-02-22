#include "gz_client.hpp"

#include <boost/bind/bind.hpp>
#include <boost/function.hpp>

#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/sim_clock.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>
#include <synapse_tinyframe/SynapseTopics.h>

GzClient::GzClient(std::string vehicle, std::shared_ptr<TinyFrame> const& tf)
    : tf_(tf)
{
    std::string model_prefix = "/model/" + vehicle;
    std::string world_prefix = "/world/default" + model_prefix;
    std::string sensor_prefix = world_prefix + "/link/sensors/sensor";
    topic_sub_clock_ = "/clock";

    // sensors
    topic_sub_altimeter_ = sensor_prefix + "/altimeter_sensor/altimeter";
    topic_sub_imu_ = sensor_prefix + "/imu_sensor/imu";
    topic_sub_magnetometer_ = sensor_prefix + "/mag_sensor/magnetometer";
    topic_sub_navsat_ = sensor_prefix + "/navsat_sensor/navsat";
    topic_sub_wheel_odometry_ = world_prefix + "/joint_state";
    topic_sub_odometry_ = model_prefix + "/odometry";

    // actuators
    topic_pub_actuators_ = "/actuators";

    // model prefix
    topic_sub_battery_state_ = model_prefix + "/battery/linear_battery/state";

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

    // actuators pub
    pub_actuators_ = Advertise<gz::msgs::Actuators>(topic_pub_actuators_);
    if (!pub_actuators_) {
        throw std::runtime_error("Error advertising topic " + topic_pub_actuators_);
    } else {
        std::cout << "publishing to " << topic_pub_actuators_ << std::endl;
    }
}

void GzClient::tf_send(TF_Msg& frame)
{
    TF_Send(tf_.get(), &frame);
}

void GzClient::handle_Clock(const gz::msgs::Clock& msg)
{
    // construct message
    // syn msg 1-to-1 with gazebo, can pass directly

    // serialize message
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize SimClock" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_SIM_CLOCK_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_Magnetometer(const gz::msgs::Magnetometer& msg)
{
    // construct message
    synapse::msgs::MagneticField syn_msg;
    syn_msg.mutable_magnetic_field()->set_x(msg.field_tesla().x());
    syn_msg.mutable_magnetic_field()->set_y(msg.field_tesla().y());
    syn_msg.mutable_magnetic_field()->set_z(msg.field_tesla().z());

    // serialize message
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Magnetometer" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_MAGNETIC_FIELD_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_IMU(const gz::msgs::IMU& msg)
{
    // construct message
    synapse::msgs::Imu syn_msg;
    syn_msg.mutable_linear_acceleration()->set_x(msg.linear_acceleration().x());
    syn_msg.mutable_linear_acceleration()->set_y(msg.linear_acceleration().y());
    syn_msg.mutable_linear_acceleration()->set_z(msg.linear_acceleration().z());
    syn_msg.mutable_angular_velocity()->set_x(msg.angular_velocity().x());
    syn_msg.mutable_angular_velocity()->set_y(msg.angular_velocity().y());
    syn_msg.mutable_angular_velocity()->set_z(msg.angular_velocity().z());

    // serialize message
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize IMU" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_IMU_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_NavSat(const gz::msgs::NavSat& msg)
{
    // construct message
    synapse::msgs::NavSatFix syn_msg;
    syn_msg.mutable_header()->set_frame_id(msg.frame_id());
    syn_msg.mutable_header()->mutable_stamp()->set_sec(
        msg.header().stamp().sec());
    syn_msg.mutable_header()->mutable_stamp()->set_nanosec(
        msg.header().stamp().nsec());
    syn_msg.set_latitude(msg.latitude_deg());
    syn_msg.set_longitude(msg.longitude_deg());
    syn_msg.set_altitude(msg.altitude());

    // serialize message
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize NavSat" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_NAV_SAT_FIX_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_Altimeter(const gz::msgs::Altimeter& msg)
{
    // construct message
    // syn msg 1-to-1 with gazebo, can pass directly

    // serialize message
    std::string data;
    if (!msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Altimeter" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_ALTIMETER_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_BatteryState(const gz::msgs::BatteryState& msg)
{
    // construct message
    static std::map<gz::msgs::BatteryState::PowerSupplyStatus, synapse::msgs::BatteryState::PowerSupplyStatus> power_supply_status_map = {
        { gz::msgs::BatteryState_PowerSupplyStatus_UNKNOWN, synapse::msgs::BatteryState_PowerSupplyStatus_UNKNOWN_STATUS },
        { gz::msgs::BatteryState_PowerSupplyStatus_CHARGING, synapse::msgs::BatteryState_PowerSupplyStatus_CHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_DISCHARGING, synapse::msgs::BatteryState_PowerSupplyStatus_DISCHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_NOT_CHARGING, synapse::msgs::BatteryState_PowerSupplyStatus_NOT_CHARGING },
        { gz::msgs::BatteryState_PowerSupplyStatus_FULL, synapse::msgs::BatteryState_PowerSupplyStatus_FULL }
    };
    synapse::msgs::BatteryState syn_msg;
    syn_msg.set_voltage(msg.voltage());
    syn_msg.set_current(msg.current());
    syn_msg.set_charge(msg.charge());
    syn_msg.set_capacity(msg.capacity());
    syn_msg.set_percentage(msg.percentage());
    syn_msg.set_power_supply_status(power_supply_status_map[msg.power_supply_status()]);

    // serialize message
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize BatteryState" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_BATTERY_STATE_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_WheelOdometry(const gz::msgs::Model& msg)
{
    // construct message
    synapse::msgs::WheelOdometry syn_msg;
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
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize WheelOdometry" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_WHEEL_ODOMETRY_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

void GzClient::handle_Odometry(const gz::msgs::Odometry& msg)
{
    // construct message
    synapse::msgs::Odometry syn_msg;

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
    std::string data;
    if (!syn_msg.SerializeToString(&data)) {
        std::cerr << "Failed to serialize Odometry" << std::endl;
        return;
    }

    // send message
    TF_Msg frame;
    frame.type = SYNAPSE_ODOMETRY_TOPIC;
    frame.len = data.length();
    frame.data = (const uint8_t*)data.c_str();
    tf_send(frame);
}

// vi: ts=4 sw=4 et
