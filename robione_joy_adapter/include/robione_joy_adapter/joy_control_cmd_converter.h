//
// Created by elaydin on 12/20/23.
//

#ifndef JOY_CONTROL_CMD_CONVERTER_H
#define JOY_CONTROL_CMD_CONVERTER_H

#include <sensor_msgs/msg/joy.hpp>
#include <robione_ros2_driver/msg/control_cmd.hpp>
#include <robione_ros2_driver/msg/control_info.hpp>
#include <robione_ros2_driver/msg/vehicle_info.hpp>

namespace robione_joy_adapter{
    class RisingEdgeDetector {
    public:
        RisingEdgeDetector() : previousState(false) {}

        bool detectRisingEdge(bool currentState) {
            bool risingEdge = false;

            if (currentState && !previousState) {
                // Current state is high, but previous state was low
                risingEdge = true;
            }

            // Update the previous state for the next detection
            previousState = currentState;

            return risingEdge;
        }

    private:
        bool previousState;
    };

    class EdgeDetector {
    public:
        EdgeDetector() : previousState(false), edge(false) {}

        bool detectEdge(bool currentState) {
            if (currentState && !previousState) {
                // Current state is high, but previous state was low
                edge = !edge;
            }

            // Update the previous state for the next detection
            previousState = currentState;

            return edge;
        }

    private:
        bool previousState;
        bool edge;
    };

    class JoyControlCmdConverter {
    private:
        EdgeDetector left_signal_detector_{};
        EdgeDetector right_signal_detector_{};
        EdgeDetector short_lamp_detector_{};
        EdgeDetector long_lamp_detector_{};
        RisingEdgeDetector flasher_detector_{};
        RisingEdgeDetector fog_detector_{};
        RisingEdgeDetector vel_increase_detector_{};
        RisingEdgeDetector vel_decrease_detector_{};

        float rate_;
        robione_ros2_driver::msg::ControlCmd control_cmd_;
        float max_velocity_;
        float vel_rate_;
        float max_steering_angle_;
        float max_tire_angle_;
        static float scale(float value, float min_in, float max_in, float min_out, float max_out);
        float tire_angle_to_scaled_steer_angle(float tire_angle_rad);
    public:
        JoyControlCmdConverter();
        ~JoyControlCmdConverter() = default;
        void UpdateJoy(const sensor_msgs::msg::Joy::SharedPtr msg, const robione_ros2_driver::msg::ControlInfo::SharedPtr control_info_ptr, const robione_ros2_driver::msg::VehicleInfo::SharedPtr vehicle_info_ptr);
        void LoadSettings(float max_velocity, float vel_rate, float max_steering_angle, float max_tire_angle);
        void set_emergency();
        robione_ros2_driver::msg::ControlCmd GetControlCmd(){return control_cmd_;};
    };
};  // namespace robione_joy_adapter

#endif //JOY_CAN_CONVERTER_H
