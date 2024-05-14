#ifndef JOY_ADAPTER_H
#define JOY_ADAPTER_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robione_joy_adapter/joy_control_cmd_converter.h>
#include <robione_ros2_driver/msg/control_cmd.hpp>
#include <robione_ros2_driver/msg/control_info.hpp>
#include <robione_ros2_driver/msg/vehicle_info.hpp>


namespace robione_joy_adapter{

    class RobioneJoyAdapter : public rclcpp::Node
    { 
    public:
        RobioneJoyAdapter();
        ~RobioneJoyAdapter() = default;
        struct configs{
            struct{
                std::string joy;
                std::string control_cmd;
                std::string control_info;
                std::string vehicle_info;
            }topic;

            struct{
                float joy_timeout;
            }timeout;
            struct{
                float vel_increase_decrease_rate;
                float max_speed;
                uint32_t control_cmd_period;
                float max_steering_angle;
                float max_tire_angle;
            }settings;
        };
        configs configs_;        
    private:
        JoyControlCmdConverter joy_converter_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time joy_time_;
        rclcpp::Publisher<robione_ros2_driver::msg::ControlCmd>::SharedPtr pub_control_cmd_;
        rclcpp::Subscription<robione_ros2_driver::msg::ControlInfo>::SharedPtr sub_control_info_;
        rclcpp::Subscription<robione_ros2_driver::msg::VehicleInfo>::SharedPtr sub_vehicle_info_;


        robione_ros2_driver::msg::ControlInfo::SharedPtr control_info_ptr_;
        robione_ros2_driver::msg::VehicleInfo::SharedPtr vehicle_info_ptr_;

        void control_info_callback(const robione_ros2_driver::msg::ControlInfo::SharedPtr msg);
        void vehicle_info_callback(const robione_ros2_driver::msg::VehicleInfo::SharedPtr msg);
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void load_parameters();
        void timer_callback();
    };

};
#endif