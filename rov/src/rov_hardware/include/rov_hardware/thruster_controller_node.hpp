// src/rov_hardware/include/rov_hardware/thruster_controller_node.hpp
#ifndef THRUSTER_CONTROLLER_NODE_HPP_
#define THRUSTER_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>

class ThrusterControllerNode : public rclcpp::Node {
public:
    ThrusterControllerNode();
    virtual ~ThrusterControllerNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void armEscs();
    void stopAllMotors();
    
    // Fonctions utilitaires pour écrire dans le PWM Linux (Sysfs)
    void initSysfsPwm(int pwm_id);
    void setPwmPulseWidth(int pwm_id, int pulse_width_us);

    // Les 4 canaux PWM matériels du Raspberry Pi (0, 1, 2, 3)
    const int MTR_HORIZ_G = 2;  // Moteur Horizontal Gauche
    const int MTR_HORIZ_D = 3;  // Moteur Horizontal Droit
    const int MTR_VERT_AV  = 0;  // Moteur Vertical Avant
    const int MTR_VERT_AR  = 1;  // Moteur Vertical Arrière

    // Paramètres du signal pour tes ESC
    const int PWM_NEUTRE = 1500;
    const int PWM_MAX_AVANT = 1900;
    const int PWM_MAX_ARRIERE = 1100;
    const int PWM_PERIOD_NS = 20000000; // 20ms (50Hz)

    std::string pwm_path_ = "/sys/class/pwm/pwmchip0/";
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

#endif // THRUSTER_CONTROLLER_NODE_HPP_