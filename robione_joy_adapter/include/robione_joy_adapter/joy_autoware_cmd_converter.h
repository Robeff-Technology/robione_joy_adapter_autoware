//
// Created by elaydin on 12/20/23.
//

#ifndef JOY_CONTROL_CMD_CONVERTER_H
#define JOY_CONTROL_CMD_CONVERTER_H

#include <sensor_msgs/msg/joy.hpp>

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
        float max_velocity_;
        float vel_rate_;
        float max_steering_angle_;
        float max_tire_angle_;
        static float scale(float value, float min_in, float max_in, float min_out, float max_out);
        float tire_angle_to_scaled_steer_angle(float tire_angle_rad);

        autoware_auto_system_msgs::msg::AutowareState autoware_state_;
        autoware_auto_control_msgs::msg::AckermannControlCommand control_cmd_;
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicators_cmd_;
        autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard_lights_cmd_;
        autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd_;
        tier4_vehicle_msgs::msg::VehicleEmergencyStamped vehicle_emergency_;
        tier4_control_msgs::msg::GateMode gate_mode_;
        autoware_auto_vehicle_msgs::msg::Engage engage_;
        autoware_auto_system_msgs::msg::EmergencyState emergency_state_;
        autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state_;
        autoware_adapi_v1_msgs::msg::RouteState route_state_;

    public:
        JoyControlCmdConverter();
        ~JoyControlCmdConverter() = default;
        void LoadSettings(float max_velocity, float vel_rate, float max_steering_angle, float max_tire_angle);
        void set_emergency();
        void uptade_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
        autoware_auto_system_msgs::msg::AutowareState get_autoware_state(){ return autoware_state_;};
        autoware_auto_control_msgs::msg::AckermannControlCommand get_control_cmd(){ return control_cmd_;};
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand get_turn_indicators_cmd(){ return turn_indicators_cmd_;};
        autoware_auto_vehicle_msgs::msg::HazardLightsCommand get_hazard_lights_cmd(){ return hazard_lights_cmd_;};
        autoware_auto_vehicle_msgs::msg::GearCommand get_gear_cmd(){ return gear_cmd_;};
        tier4_vehicle_msgs::msg::VehicleEmergencyStamped get_vehicle_emergency(){ return vehicle_emergency_;};
        tier4_control_msgs::msg::GateMode get_gate_mode(){ return gate_mode_;};
        autoware_auto_vehicle_msgs::msg::Engage get_engage(){ return engage_;};
        autoware_auto_system_msgs::msg::EmergencyState get_emergency_state(){ return emergency_state_;};
        autoware_adapi_v1_msgs::msg::OperationModeState get_operation_mode_state(){ return operation_mode_state_;};
        autoware_adapi_v1_msgs::msg::RouteState get_route_state(){ return route_state_;};
    };
};  // namespace robione_joy_adapter

#endif //JOY_CAN_CONVERTER_H
