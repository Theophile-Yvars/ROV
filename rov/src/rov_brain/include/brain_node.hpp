#pragma once
#include <rclcpp/rclcpp.hpp>

class BrainNode : public rclcpp::Node{
private:
    void run();
public:
    BrainNode();
};
