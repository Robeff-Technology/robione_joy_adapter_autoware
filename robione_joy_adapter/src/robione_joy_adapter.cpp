#include "robione_joy_adapter/robione_joy_adapter.h"

namespace robione_joy_adapter{
    RobioneJoyAdapter::RobioneJoyAdapter(const rclcpp::NodeOptions& options): Node("robione_joy_adapter", options)
    {
        load_parameters();
        create_autoware_publishers();
        joy_converter_.LoadSettings(configs_.settings.max_speed, configs_.settings.vel_increase_decrease_rate, configs_.settings.max_steering_angle, configs_.settings.max_tire_angle);
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(configs_.topic.joy, 10, std::bind(&RobioneJoyAdapter::joy_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(configs_.settings.control_cmd_period), std::bind(&RobioneJoyAdapter::timer_callback, this));
    }

    void RobioneJoyAdapter::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        (void)msg;
        joy_time_ = rclcpp::Clock().now();
    }

    void RobioneJoyAdapter::load_parameters()
    {
        configs_.topic.joy = this->declare_parameter<std::string>("topic_config.joystick_topic", "/joy");
        configs_.topic.control_cmd = this->declare_parameter<std::string>("topic_config.control_cmd_topic", "/control_cmd");
        configs_.topic.control_info = this->declare_parameter<std::string>("topic_config.control_info_topic", "/control_info");
        configs_.topic.vehicle_info = this->declare_parameter<std::string>("topic_config.vehicle_info_topic", "/vehicle_info");
        configs_.timeout.joy_timeout = this->declare_parameter<float>("timeout_config.joystick_timeout", 5.0);
        configs_.settings.vel_increase_decrease_rate = this->declare_parameter<double>("settings.vel_increase_decrease_rate", 0.5);
        configs_.settings.max_speed = this->declare_parameter<double>("settings.max_velocity", 5.0);
        configs_.settings.control_cmd_period = this->declare_parameter<int>("settings.control_cmd_period", 200);
        configs_.settings.max_steering_angle = this->declare_parameter<double>("settings.max_steer_angle", 300.0);
        configs_.settings.max_tire_angle = this->declare_parameter<double>("settings.max_tire_angle", 0.45);

        RCLCPP_INFO(this->get_logger(), "--------------------------------CONFIGS--------------------------------");
        RCLCPP_INFO(this->get_logger(), "Joy topic: %s", configs_.topic.joy.c_str());
        RCLCPP_INFO(this->get_logger(), "Control cmd topic: %s", configs_.topic.control_cmd.c_str());
        RCLCPP_INFO(this->get_logger(), "Control info topic: %s", configs_.topic.control_info.c_str());
        RCLCPP_INFO(this->get_logger(), "Vehicle info topic: %s", configs_.topic.vehicle_info.c_str());
        RCLCPP_INFO(this->get_logger(), "Joy timeout: %f", configs_.timeout.joy_timeout);
        RCLCPP_INFO(this->get_logger(), "Vel increase decrease rate: %f", configs_.settings.vel_increase_decrease_rate);
        RCLCPP_INFO(this->get_logger(), "Max speed: %f", configs_.settings.max_speed);
        RCLCPP_INFO(this->get_logger(), "Control cmd period: %d", configs_.settings.control_cmd_period);
        RCLCPP_INFO(this->get_logger(), "Max steering angle: %f", configs_.settings.max_steering_angle);
        RCLCPP_INFO(this->get_logger(), "Max tire angle: %f", configs_.settings.max_tire_angle);
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
    }

    void RobioneJoyAdapter::timer_callback()
    {
        if(rclcpp::Clock().now() - joy_time_ > rclcpp::Duration::from_seconds(configs_.timeout.joy_timeout))
        {
            RCLCPP_WARN(this->get_logger(), "Joystick timeout");
            return;
        }
    }

    void RobioneJoyAdapter::create_autoware_publishers()
    {
        pub_autoware_state_ = this->create_publisher<autoware_auto_system_msgs::msg::AutowareState>("/autoware/state", 10);
        pub_control_cmd_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 10);
        pub_turn_indicators_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", 10);
        pub_hazard_lights_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>("/control/command/hazard_lights_cmd", 10);
        pub_gear_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 10);
        pub_vehicle_emergency_ = this->create_publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>("/control/command/emergency_cmd", 10);
        pub_gate_mode_ = this->create_publisher<tier4_control_msgs::msg::GateMode>("/control/current_gate_mode", 10);
        pub_engage_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("/autoware/engage", 10);
        pub_emergency_state_ = this->create_publisher<autoware_auto_system_msgs::msg::EmergencyState>("/system/emergency/emergency_state", 10);
        pub_operation_mode_state_ = this->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state", 10);
        pub_route_state_ = this->create_publisher<autoware_adapi_v1_msgs::msg::RouteState>("/api/routing/state", 10);
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robione_joy_adapter::RobioneJoyAdapter)