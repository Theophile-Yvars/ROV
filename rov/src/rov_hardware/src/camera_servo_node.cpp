#include "rov_hardware/camera_servo_node.hpp"
#include <algorithm> // Pour std::max et std::min

CameraServoNode::CameraServoNode() : Node("camera_servo") {
    // 0 correspond au chip GPIO par défaut (GPIOchip 0)
    h_ = lgGpiochipOpen(0);
    if (h_ < 0) {
        RCLCPP_FATAL(this->get_logger(), "Échec d'ouverture du chip GPIO !");
        return;
    }

    // Configure la broche en sortie
    if (lgGpioClaimOutput(h_, 0, gpio_pin_, 0) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Échec de configuration du GPIO %d", gpio_pin_);
    }

    sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/rov/camera_tilt", 10, std::bind(&CameraServoNode::servo_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node Servo Caméra démarré avec lgpio sur GPIO 27.");
}

CameraServoNode::~CameraServoNode() {
    // Arrêt du signal servo et fermeture du chip
    lgTxServo(h_, gpio_pin_, 0, 50, 0, 0); 
    lgGpiochipClose(h_);
}

void CameraServoNode::servo_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    set_servo_angle(msg->data);
}

void CameraServoNode::set_servo_angle(float val) {
    // Conversion : -1.0 -> 500us, 1.0 -> 2500us
    int pulse_width = 1500 + static_cast<int>(val * 1000);
    pulse_width = std::max(500, std::min(2500, pulse_width));
    
    // lgTxServo(handle, gpio, pulse_width, frequency, frame_ms, flags)
    // 50Hz est la fréquence standard pour les servomoteurs
    lgTxServo(h_, gpio_pin_, pulse_width, 50, 0, 0);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraServoNode>());
    rclcpp::shutdown();
    return 0;
}