// camera_servo_node.hpp
#ifndef CAMERA_SERVO_NODE_HPP
#define CAMERA_SERVO_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <lgpio.h>

class CameraServoNode : public rclcpp::Node {
public:
    CameraServoNode();
    ~CameraServoNode();

private:
    void servo_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void move_step();
    void stop_servo();

    int h_ = -1;
    static constexpr int   GPIO_PIN     = 27;
    static constexpr int   PULSE_MIN    = 500;
    static constexpr int   PULSE_MAX    = 2500;
    static constexpr int   PULSE_CENTER = 1500;

    // Vitesse max en µs par tick (20ms)
    // 30 µs/tick × 50 ticks/s = 1500 µs/s → pleine course en ~1.3s
    static constexpr float MAX_SPEED = 30.0f;

    // Watchdog : coupure si silence > Xms
    static constexpr int WATCHDOG_MS = 200;

    float current_pulse_  = PULSE_CENTER;
    float velocity_       = 0.0f;   // reçue du topic, entre -1.0 et +1.0
    int   last_sent_pulse_ = -1;

    rclcpp::Time last_command_time_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_move_;
};

#endif