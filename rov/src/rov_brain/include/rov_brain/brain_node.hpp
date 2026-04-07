#ifndef BRAIN_NODE_HPP_
#define BRAIN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class BrainNode : public rclcpp::Node {
public:
    BrainNode();

private:
    // Fonction appelée à intervalle régulier (Boucle de contrôle)
    void control_loop();

    // Fonction appelée à chaque réception de température
    void temp_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Objets ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temp_sub_;

    // Données du ROV
    float current_temperature_ = 0.0f;
    int cpt_ = 0; 
};

#endif // BRAIN_NODE_HPP_