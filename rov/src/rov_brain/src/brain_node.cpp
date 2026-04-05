#include "rov_brain/brain_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

BrainNode::BrainNode() : Node("brain") {
    RCLCPP_INFO(this->get_logger(), "Brain node started");
    timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&BrainNode::control_loop, this)
    );
}

void BrainNode::control_loop() {
    RCLCPP_INFO(this->get_logger(), "Brain node running - Calculating ROV state...");
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<BrainNode>();    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}