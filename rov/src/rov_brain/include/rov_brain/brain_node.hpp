#ifndef BRAIN_NODE_HPP_
#define BRAIN_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp" 
#include "geometry_msgs/msg/vector3.hpp" // Nouveau : Requis pour les angles d'Euler de l'IMU

class BrainNode : public rclcpp::Node {
public:
    BrainNode();

private:
    // Callbacks
    void temp_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void cpu_temp_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void depth_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void water_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
    void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg); 
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg); // Nouveau

    // Boucle de contrôle et timer
    void control_loop();
    rclcpp::TimerBase::SharedPtr timer_;

    // Abonnements (Subscriptions)
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpu_temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr water_temp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_; // Nouveau

    // Variables de stockage
    float current_temperature_;  // Interne caisson (TMP102)
    float cpu_temperature_;      // Interne puce Pi 5
    double current_depth_;       // Externe (MS5837)
    double water_temperature_;   // Externe (MS5837)
    double current_pressure_;    // Externe (MS5837) 
    
    // Nouveau : Stockage de l'orientation (x: Roll, y: Pitch, z: Heading/Yaw)
    geometry_msgs::msg::Vector3 current_orientation_; 

    int cpt_;
};

#endif // BRAIN_NODE_HPP_