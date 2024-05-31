#ifndef JOY_ADAPTER_H
#define JOY_ADAPTER_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robione_joy_adapter/joy_autoware_cmd_converter.h>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>




namespace robione_joy_adapter{

    class RobioneJoyAdapter : public rclcpp::Node
    { 
    public:
        RobioneJoyAdapter(const rclcpp::NodeOptions & options);
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

        rclcpp::Publisher<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;
        rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_control_cmd_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr pub_hazard_lights_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
        rclcpp::Publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr pub_vehicle_emergency_;
        rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr pub_engage_;
        rclcpp::Publisher<autoware_auto_system_msgs::msg::EmergencyState>::SharedPtr pub_emergency_state_;
        rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_state_;
        rclcpp::Publisher<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr pub_route_state_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void create_autoware_publishers();
        void load_parameters();
        void timer_callback();
    };

};
#endif