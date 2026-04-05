#pragma once
#include <rclcpp/rclcpp.hpp>

class BrainNode : public rclcpp::Node {
public:
    BrainNode();
private:
    void control_loop();
    rclcpp::TimerBase::SharedPtr timer_;
};