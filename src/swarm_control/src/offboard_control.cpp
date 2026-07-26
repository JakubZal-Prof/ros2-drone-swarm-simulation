#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <vector>
#include <array>
#include <cmath>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        this->declare_parameter<std::string>("role", "leader");
        this->declare_parameter<int>("target_system", 1);
        this->declare_parameter<std::string>("leader_position_topic", "");
        this->declare_parameter<double>("offset_x", 0.0);
        this->declare_parameter<double>("offset_y", 0.0);
        this->declare_parameter<double>("offset_z", 0.0);
        this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{0.0, 0.0});
        this->declare_parameter<double>("cruise_altitude", -5.0);
        this->declare_parameter<double>("land_hold_seconds", 5.0);

        role_ = this->get_parameter("role").as_string();
        target_system_ = this->get_parameter("target_system").as_int();
        leader_topic_ = this->get_parameter("leader_position_topic").as_string();
        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();
        offset_z_ = this->get_parameter("offset_z").as_double();
        cruise_altitude_ = this->get_parameter("cruise_altitude").as_double();
        land_hold_seconds_ = this->get_parameter("land_hold_seconds").as_double();

        std::vector<double> wp_flat = this->get_parameter("waypoints").as_double_array();
        for (size_t i = 0; i + 1 < wp_flat.size(); i += 2) {
            waypoints_.push_back({wp_flat[i], wp_flat[i+1]});
        }

        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
            "fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
            "fmu/in/vehicle_command", 10);

        vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
            "fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(),
            [this](const VehicleStatus::SharedPtr msg) {
                is_armed_ = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
            });

        local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
            [this](const VehicleLocalPosition::SharedPtr msg) {
                my_x_ = msg->x; my_y_ = msg->y; my_z_ = msg->z;
            });

        if (role_ == "follower" && !leader_topic_.empty()) {
            leader_position_sub_ = this->create_subscription<VehicleLocalPosition>(
                leader_topic_, rclcpp::QoS(10).best_effort(),
                [this](const VehicleLocalPosition::SharedPtr msg) {
                    leader_x_ = msg->x; leader_y_ = msg->y; leader_z_ = msg->z;
                    leader_position_known_ = true;
                });
        }

        timer_ = this->create_wall_timer(
            100ms, std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Węzeł wystartował, rola: %s", role_.c_str());
    }

private:
    enum class MissionState { CRUISE_TO_ZONE, LANDING, HOLDING, TAKEOFF };

    void timer_callback()
    {
        if (counter_ >= 50 && !is_armed_ && counter_ % 20 == 0) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }

        publish_offboard_control_mode();

        if (role_ == "leader") {
            update_leader_target();
        } else {
            update_follower_target();
        }

        counter_++;
    }

    void update_leader_target()
    {
        if (waypoints_.empty()) return;

        auto& wp = waypoints_[current_wp_index_];

        switch (mission_state_) {
            case MissionState::CRUISE_TO_ZONE: {
                publish_trajectory_setpoint(wp[0], wp[1], cruise_altitude_);
                double dist = std::sqrt(std::pow(my_x_ - wp[0], 2) + std::pow(my_y_ - wp[1], 2));
                if (dist < 0.5) {
                    mission_state_ = MissionState::LANDING;
                    RCLCPP_INFO(this->get_logger(), "Dotarto nad strefę %zu, ląduję", current_wp_index_);
                }
                break;
            }
            case MissionState::LANDING: {
                publish_trajectory_setpoint(wp[0], wp[1], -0.15);
                if (std::abs(my_z_ - (-0.15)) < 0.2) {
                    mission_state_ = MissionState::HOLDING;
                    hold_start_counter_ = counter_;
                    RCLCPP_INFO(this->get_logger(), "Wylądowano w strefie %zu, czekam", current_wp_index_);
                }
                break;
            }
            case MissionState::HOLDING: {
                publish_trajectory_setpoint(wp[0], wp[1], -0.15);
                double elapsed = (counter_ - hold_start_counter_) * 0.1;
                if (elapsed >= land_hold_seconds_) {
                    mission_state_ = MissionState::TAKEOFF;
                    RCLCPP_INFO(this->get_logger(), "Startuję ze strefy %zu", current_wp_index_);
                }
                break;
            }
            case MissionState::TAKEOFF: {
                publish_trajectory_setpoint(wp[0], wp[1], cruise_altitude_);
                if (std::abs(my_z_ - cruise_altitude_) < 0.3) {
                    current_wp_index_ = (current_wp_index_ + 1) % waypoints_.size();
                    mission_state_ = MissionState::CRUISE_TO_ZONE;
                    RCLCPP_INFO(this->get_logger(), "Lecę do kolejnej strefy (%zu)", current_wp_index_);
                }
                break;
            }
        }
    }

    void update_follower_target()
    {
        if (!leader_position_known_) {
            publish_trajectory_setpoint(offset_x_, offset_y_, offset_z_);
            return;
        }
        publish_trajectory_setpoint(leader_x_ + offset_x_,
                                     leader_y_ + offset_y_,
                                     leader_z_ + offset_z_);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0);
        RCLCPP_INFO(this->get_logger(), "Wysłano komendę uzbrojenia (próba, counter=%lu)", counter_);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(double x, double y, double z)
    {
        TrajectorySetpoint msg{};
        msg.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = target_system_;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr leader_position_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t counter_ = 0;
    bool is_armed_ = false;
    int target_system_;
    std::string role_;
    std::string leader_topic_;

    double my_x_ = 0.0, my_y_ = 0.0, my_z_ = 0.0;
    double leader_x_ = 0.0, leader_y_ = 0.0, leader_z_ = 0.0;
    bool leader_position_known_ = false;
    double offset_x_, offset_y_, offset_z_;

    MissionState mission_state_ = MissionState::CRUISE_TO_ZONE;
    uint64_t hold_start_counter_ = 0;
    double cruise_altitude_;
    double land_hold_seconds_;

    std::vector<std::array<double,2>> waypoints_;
    size_t current_wp_index_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
