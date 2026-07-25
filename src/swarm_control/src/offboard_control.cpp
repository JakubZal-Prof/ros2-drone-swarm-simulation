#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <chrono>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        this->declare_parameter<double>("target_x", 0.0);
        this->declare_parameter<double>("target_y", 0.0); 
	this->declare_parameter<int>("target_system", 1);
        target_system_ = this->get_parameter("target_system").as_int();
        this->declare_parameter<double>("target_z", -5.0);

        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        target_z_ = this->get_parameter("target_z").as_double();

        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
            "fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
            "fmu/in/vehicle_command", 10);

        // Subskrybujemy status drona, żeby wiedzieć czy jest już uzbrojony
        vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
            "fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(),
            [this](const VehicleStatus::SharedPtr msg) {
                is_armed_ = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
            });

        timer_ = this->create_wall_timer(
            100ms, std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Węzeł wystartował, cel: (%.1f, %.1f, %.1f)",
                    target_x_, target_y_, target_z_);
    }

private:
    void timer_callback()
    {
        // Po 5 sekundach "rozgrzewki", próbuj uzbroić co 2 sekundy, DOPÓKI się nie uda
        if (counter_ >= 50 && !is_armed_ && counter_ % 20 == 0) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }

        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        counter_++;
    }

    void arm()
    {
        // param2 = 21196 to magiczna wartość PX4 oznaczająca "wymuś uzbrojenie",
        // pomija część "miękkich" zabezpieczeń (jak brak połączenia z GCS) - używana w symulacji/testach
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0);
        RCLCPP_INFO(this->get_logger(), "Wysłano komendę uzbrojenia (próba, counter=%lu)", counter_);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {static_cast<float>(target_x_),
                         static_cast<float>(target_y_),
                         static_cast<float>(target_z_)};
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
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t counter_ = 0;
    bool is_armed_ = false;
    double target_x_, target_y_, target_z_;
    int target_system_;
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
