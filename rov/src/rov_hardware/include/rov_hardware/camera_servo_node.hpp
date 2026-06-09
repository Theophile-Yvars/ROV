#ifndef CAMERA_SERVO_NODE_HPP_
#define CAMERA_SERVO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <lgpio.h> // Nouvelle bibliothèque

class CameraServoNode : public rclcpp::Node {
public:
    CameraServoNode();
    ~CameraServoNode();

private:
    void servo_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void set_servo_angle(float normalized_val);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    int h_; // Handle pour le chip GPIO
    const int gpio_pin_ = 27;
};

#endif