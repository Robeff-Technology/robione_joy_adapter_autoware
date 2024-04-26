//
// Created by elaydin on 12/20/23.
//

#include "robione_joy_adapter/joy_control_cmd_converter.h"
#include <cmath>
namespace robione_joy_adapter{

    JoyControlCmdConverter::JoyControlCmdConverter() {
        control_cmd_.steer_angle = 0.0;
        control_cmd_.head_light = robione_ros2_driver::msg::ControlCmd::HEAD_LIGHT_OFF;
        control_cmd_.indicators = robione_ros2_driver::msg::ControlCmd::NO_COMMAND;
        control_cmd_.emergency = robione_ros2_driver::msg::ControlCmd::EMERGENCY_ON;
        control_cmd_.handbrake = robione_ros2_driver::msg::ControlCmd::HANDBRAKE_ON;
        control_cmd_.steer_vel = 43.0f;
        max_velocity_ = 5.0; 
        vel_rate_ = 0.5;
    }

    void JoyControlCmdConverter::LoadSettings(float max_velocity, float vel_rate) {
        max_velocity_ = max_velocity;
        vel_rate_ = vel_rate;
    }

    float JoyControlCmdConverter::scale(float value, float min_in, float max_in, float min_out, float max_out) {
        return min_out + (value - min_in) * (max_out - min_out) / (max_in - min_in);
    }

    void JoyControlCmdConverter::UpdateJoy(const sensor_msgs::msg::Joy::SharedPtr msg, float actual_vel) {
        control_cmd_.steer_angle = msg->axes[0];

        control_cmd_.steer_vel = scale(std::fabs(actual_vel), 0, 12.0f, 325.0f, 80.0f);
        if(msg->buttons[9] > 0){
            control_cmd_.emergency = robione_ros2_driver::msg::ControlCmd::EMERGENCY_OFF;
        }
        if(msg->buttons[8] > 0){
            control_cmd_.emergency = robione_ros2_driver::msg::ControlCmd::EMERGENCY_ON;
        }

        if(msg->axes[5] >= 1.0f){
            control_cmd_.handbrake = robione_ros2_driver::msg::ControlCmd::HANDBRAKE_ON;
        }
        else if(msg->axes[5] <= -1.0f){
            control_cmd_.handbrake = robione_ros2_driver::msg::ControlCmd::HANDBRAKE_OFF;
        }

        if(control_cmd_.emergency != robione_ros2_driver::msg::ControlCmd::EMERGENCY_ON){
            if(vel_increase_detector_.detectRisingEdge(msg->buttons[3] > 0)){
                control_cmd_.set_velocity = std::min(control_cmd_.set_velocity + vel_rate_, max_velocity_);
            }
            if(vel_decrease_detector_.detectRisingEdge(msg->buttons[1] > 0)){
                control_cmd_.set_velocity = std::max(control_cmd_.set_velocity - vel_rate_, -max_velocity_);
            }

            if(msg->buttons[2] > 0){
                control_cmd_.set_velocity = 0.0;
            }

        }
        else{
            control_cmd_.set_velocity = 0.0;
        }


        if(left_signal_detector_.detectEdge(msg->buttons[4] > 0)){
            if(control_cmd_.indicators == robione_ros2_driver::msg::ControlCmd::NO_COMMAND){
                control_cmd_.indicators = robione_ros2_driver::msg::ControlCmd::LEFT_TURN_SIGNAL;
            }
        }
        else{
            if(control_cmd_.indicators == robione_ros2_driver::msg::ControlCmd::LEFT_TURN_SIGNAL){
                control_cmd_.indicators = robione_ros2_driver::msg::ControlCmd::NO_COMMAND;
            }
        }

        if(right_signal_detector_.detectEdge(msg->buttons[5] > 0)){
            if(control_cmd_.indicators == robione_ros2_driver::msg::ControlCmd::NO_COMMAND){
                control_cmd_.indicators = robione_ros2_driver::msg::ControlCmd::RIGHT_TURN_SIGNAL;
            }
        }
        else{
            if(control_cmd_.indicators == robione_ros2_driver::msg::ControlCmd::RIGHT_TURN_SIGNAL){
                control_cmd_.indicators = robione_ros2_driver::msg::ControlCmd::NO_COMMAND;
            }
        }

        if(short_lamp_detector_.detectEdge(msg->buttons[6] > 0)){
            if(control_cmd_.head_light != robione_ros2_driver::msg::ControlCmd::LOW_HEAD_LIGHT_ON){
                control_cmd_.head_light = robione_ros2_driver::msg::ControlCmd::LOW_HEAD_LIGHT_ON;
            }
        }
        else{
            if(control_cmd_.head_light == robione_ros2_driver::msg::ControlCmd::LOW_HEAD_LIGHT_ON){
                control_cmd_.head_light = robione_ros2_driver::msg::ControlCmd::HEAD_LIGHT_OFF;
            }
        }
        if(long_lamp_detector_.detectEdge(msg->buttons[7] > 0)){
            if(control_cmd_.head_light != robione_ros2_driver::msg::ControlCmd::HIGH_HEAD_LIGHT_ON){
                control_cmd_.head_light = robione_ros2_driver::msg::ControlCmd::HIGH_HEAD_LIGHT_ON;
            }
        }
        else{
            if(control_cmd_.head_light == robione_ros2_driver::msg::ControlCmd::HIGH_HEAD_LIGHT_ON){
                control_cmd_.head_light = robione_ros2_driver::msg::ControlCmd::HEAD_LIGHT_OFF;
            }
        }
    }
};  // namespace robione_joy_adapter