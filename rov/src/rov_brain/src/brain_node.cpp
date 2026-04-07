#include "rov_brain/brain_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

BrainNode::BrainNode() : Node("brain") {
    RCLCPP_INFO(this->get_logger(), "Cerveau du ROV démarré.");
    cpt_ = 0; 

    // 1. On s'abonne au topic de température créé par le hardware
    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "rov/temperature", 
        10, 
        std::bind(&BrainNode::temp_callback, this, std::placeholders::_1)
    );

    // 2. On lance la boucle de contrôle (toutes les 100ms = 10Hz)
    timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&BrainNode::control_loop, this)
    );
}

void BrainNode::temp_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_temperature_ = msg->data;
}

void BrainNode::control_loop() {
    cpt_++;
    if (cpt_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Température actuelle : %.2f °C", current_temperature_);
    }
    if (cpt_ == 100) {
        cpt_ = 0;
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<BrainNode>();    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}