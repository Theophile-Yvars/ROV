#ifndef BRAIN_NODE_HPP_
#define BRAIN_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp" 
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

class BrainNode : public rclcpp::Node {
public:
    BrainNode();
    ~BrainNode() = default; // Destructeur par défaut propre

private:
    // Callbacks capteurs
    void temp_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void cpu_temp_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void depth_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void water_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
    void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg); 
    void imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void cmd_vel_input_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    
    void control_loop();

    rclcpp::TimerBase::SharedPtr timer_;

    // Abonnements (plus de cam_tilt_sub_)
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpu_temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr water_temp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr batt_v_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;

    // Variables de stockage
    float current_temperature_;  
    float cpu_temperature_;       
    double current_depth_;        
    double water_temperature_;   
    double current_pressure_;    
    geometry_msgs::msg::Vector3 current_orientation_; 
    float battery_voltage_;
    
    geometry_msgs::msg::Twist last_pilot_command_;
    rclcpp::Time last_command_time_;
    int cpt_;
};

#endif // BRAIN_NODE_HPP_