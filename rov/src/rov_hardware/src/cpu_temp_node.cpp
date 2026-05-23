#include "rov_hardware/cpu_temp_node.hpp"
#include <fstream>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

CpuTempNode::CpuTempNode() : Node("cpu_temp_node") {
    // Topic clair pour la télémétrie du CPU
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/rov/cpu_temperature", 10);

    timer_ = this->create_wall_timer(
        2s, 
        std::bind(&CpuTempNode::publish_cpu_temp, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Nœud de surveillance de la température CPU initialisé.");
}

void CpuTempNode::publish_cpu_temp() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Impossible de lire le fichier thermal du CPU.");
        return;
    }

    double raw_temp = 0.0;
    file >> raw_temp;
    file.close();

    float cpu_temp = static_cast<float>(raw_temp / 1000.0);

    auto message = std_msgs::msg::Float32();
    message.data = cpu_temp;
    publisher_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CpuTempNode>());
    rclcpp::shutdown();
    return 0;
}