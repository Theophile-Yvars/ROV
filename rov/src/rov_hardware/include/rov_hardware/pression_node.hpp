#pragma once

#include <chrono>
#include <memory>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"

class PressionNode : public rclcpp::Node {
public:
    /**
     * @brief Constructeur du Node Pression
     */
    PressionNode();

    /**
     * @brief Destructeur pour fermer proprement le descripteur I2C
     */
    ~PressionNode();

private:
    // Attributs pour la communication I2C Linux brute
    int i2c_fd_;
    uint16_t C[8]; // Tableau pour stocker les 7 coefficients d'usine (PROM)

    // Publishers ROS 2
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    
    // Timer pour la boucle de lecture périodique
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Initialise le périphérique I2C et charge les coefficients de calibration
     */
    void init_i2c();

    /**
     * @brief Lit les données brutes du capteur, applique les compensations et publie sur les topics
     */
    void read_and_publish();
};