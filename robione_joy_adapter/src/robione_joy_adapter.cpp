#include "robione_joy_adapter/robione_joy_adapter.h"

namespace robione_joy_adapter{
    RobioneJoyAdapter::RobioneJoyAdapter(): Node("robione_joy_adapter")
    {
        load_parameters();
        joy_converter_.LoadSettings(configs_.settings.max_speed, configs_.settings.vel_increase_decrease_rate);
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(configs_.topic.joy, 10, std::bind(&RobioneJoyAdapter::joy_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(configs_.settings.control_cmd_period), std::bind(&RobioneJoyAdapter::timer_callback, this));
        pub_control_cmd_ = this->create_publisher<robione_ros2_driver::msg::ControlCmd>(configs_.topic.control_cmd, 10);
        sub_control_info_ = this->create_subscription<robione_ros2_driver::msg::ControlInfo>(configs_.topic.control_info, 10, std::bind(&RobioneJoyAdapter::control_info_callback, this, std::placeholders::_1));
    }

    void RobioneJoyAdapter::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        float vel = 0;
        if(control_info_ptr_ != nullptr)
        {
            vel = control_info_ptr_->velocity;
        }
        joy_converter_.UpdateJoy(msg, vel);
        joy_time_ = rclcpp::Clock().now();
    }

    void RobioneJoyAdapter::load_parameters()
    {
        configs_.topic.joy = this->declare_parameter<std::string>("topic_config.joystick_topic", "/joy");
        configs_.topic.control_cmd = this->declare_parameter<std::string>("topic_config.control_cmd_topic", "/control_cmd");
        configs_.topic.control_info = this->declare_parameter<std::string>("topic_config.control_info_topic", "/control_info");
        configs_.timeout.joy_timeout = this->declare_parameter<float>("timeout_config.joystick_timeout", 5.0);
        configs_.settings.vel_increase_decrease_rate = this->declare_parameter<double>("settings.vel_increase_decrease_rate", 0.5);
        configs_.settings.max_speed = this->declare_parameter<double>("settings.max_velocity", 5.0);
        configs_.settings.control_cmd_period = this->declare_parameter<int>("settings.control_cmd_period", 200);

        RCLCPP_INFO(this->get_logger(), "--------------------------------CONFIGS--------------------------------");
        RCLCPP_INFO(this->get_logger(), "Joy topic: %s", configs_.topic.joy.c_str());
        RCLCPP_INFO(this->get_logger(), "Control cmd topic: %s", configs_.topic.control_cmd.c_str());
        RCLCPP_INFO(this->get_logger(), "Control info topic: %s", configs_.topic.control_info.c_str());
        RCLCPP_INFO(this->get_logger(), "Joy timeout: %f", configs_.timeout.joy_timeout);
        RCLCPP_INFO(this->get_logger(), "Vel increase decrease rate: %f", configs_.settings.vel_increase_decrease_rate);
        RCLCPP_INFO(this->get_logger(), "Max speed: %f", configs_.settings.max_speed);
        RCLCPP_INFO(this->get_logger(), "Control cmd period: %d", configs_.settings.control_cmd_period);
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------------------------");
    }

    void RobioneJoyAdapter::timer_callback()
    {
        if(rclcpp::Clock().now() - joy_time_ > rclcpp::Duration::from_seconds(configs_.timeout.joy_timeout))
        {
            RCLCPP_WARN(this->get_logger(), "Joystick timeout");
            return;
        }
        auto control_cmd = joy_converter_.GetControlCmd();
        pub_control_cmd_->publish(control_cmd);
    }

    void RobioneJoyAdapter::control_info_callback(const robione_ros2_driver::msg::ControlInfo::SharedPtr msg)
    {
        control_info_ptr_ = msg;
    }
};