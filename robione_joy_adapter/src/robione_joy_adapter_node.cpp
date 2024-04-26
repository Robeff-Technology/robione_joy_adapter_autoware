#include <rclcpp/rclcpp.hpp>
#include <robione_joy_adapter/robione_joy_adapter.h>
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robione_joy_adapter::RobioneJoyAdapter>());
    return 0;
}