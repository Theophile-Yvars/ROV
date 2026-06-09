// camera_servo_node.cpp
#include "rov_hardware/camera_servo_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

CameraServoNode::CameraServoNode() : Node("camera_servo") {

    h_ = lgGpiochipOpen(0);
    if (h_ < 0) {
        RCLCPP_FATAL(get_logger(), "Échec d'ouverture du chip GPIO !");
        return;
    }
    if (lgGpioClaimOutput(h_, 0, GPIO_PIN, 0) < 0) {
        RCLCPP_FATAL(get_logger(), "Échec de configuration du GPIO %d", GPIO_PIN);
        return;
    }

    lgTxServo(h_, GPIO_PIN, PULSE_CENTER, 50, 0, 0);
    last_sent_pulse_   = PULSE_CENTER;
    last_command_time_ = this->now();

    sub_ = create_subscription<std_msgs::msg::Float32>(
        "/rov/camera_tilt", 1,
        std::bind(&CameraServoNode::servo_callback, this, std::placeholders::_1));

    timer_move_ = create_wall_timer(
        33ms, std::bind(&CameraServoNode::move_step, this));

    RCLCPP_INFO(get_logger(), "Node Servo Caméra démarré (mode vitesse).");
}

CameraServoNode::~CameraServoNode() {
    if (h_ >= 0) {
        lgTxServo(h_, GPIO_PIN, 0, 50, 0, 0);
        lgGpiochipClose(h_);
    }
}

void CameraServoNode::servo_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // Clamp la vitesse reçue entre -1.0 et +1.0
    velocity_ = std::clamp(msg->data, -1.0f, 1.0f);
    last_command_time_ = this->now();
}

void CameraServoNode::move_step() {

    // ── Watchdog : plus de commande depuis WATCHDOG_MS → on stoppe ──
    auto elapsed_ms = (this->now() - last_command_time_).nanoseconds() / 1'000'000;
    if (elapsed_ms > WATCHDOG_MS) {
        velocity_ = 0.0f;
        stop_servo();
        return;
    }

    // ── Intégration : position += vitesse × pas_max ──────────────
    current_pulse_ += velocity_ * MAX_SPEED;
    current_pulse_  = std::clamp(current_pulse_, (float)PULSE_MIN, (float)PULSE_MAX);

    // ── Performance : GPIO seulement si changement >= 5µs ────────
    int pulse_int = static_cast<int>(std::round(current_pulse_));
    pulse_int = std::clamp(pulse_int, PULSE_MIN, PULSE_MAX);

    if (std::abs(pulse_int - last_sent_pulse_) < 5) return;

    lgTxServo(h_, GPIO_PIN, pulse_int, 50, 0, 0);
    last_sent_pulse_ = pulse_int;
}

void CameraServoNode::stop_servo() {
    if (last_sent_pulse_ == 0) return;
    lgTxServo(h_, GPIO_PIN, 0, 50, 0, 0);
    last_sent_pulse_ = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraServoNode>());
    rclcpp::shutdown();
    return 0;
}