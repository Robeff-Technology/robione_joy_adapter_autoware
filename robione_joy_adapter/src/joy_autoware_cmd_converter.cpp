//
// Created by elaydin on 12/20/23.
//

#include "robione_joy_adapter/joy_autoware_cmd_converter.h"
#include <cmath>
namespace robione_joy_adapter{

    JoyControlCmdConverter::JoyControlCmdConverter() {
        max_velocity_ = 5.0; 
        vel_rate_ = 0.5;
    }

    void JoyControlCmdConverter::LoadSettings(float max_velocity, float vel_rate, float max_steer_angle, float max_tire_angle) {
        max_velocity_ = max_velocity;
        vel_rate_ = vel_rate;
        max_steering_angle_ = max_steer_angle;
        max_tire_angle_ = max_tire_angle;
    }

    float JoyControlCmdConverter::scale(float value, float min_in, float max_in, float min_out, float max_out) {
        return min_out + (value - min_in) * (max_out - min_out) / (max_in - min_in);
    }

    float JoyControlCmdConverter::tire_angle_to_scaled_steer_angle(float tire_angle_rad) {
        float tire_to_steer_deg = tire_angle_rad / 0.001511;
        return tire_to_steer_deg / max_steering_angle_;
    }

    void JoyControlCmdConverter::uptade_joy(const sensor_msgs::msg::Joy::SharedPtr msg){
        (void)msg;
    }

};  // namespace robione_joy_adapter