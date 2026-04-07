#ifndef TMP102_NODE_HPP_
#define TMP102_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class TMP102Node : public rclcpp::Node {
public:
    TMP102Node();
    ~TMP102Node();

private:
    // Fonctions
    void init_i2c();
    void read_temperature();

    // Variables ROS 2
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables I2C
    int fd_;
    const uint8_t I2C_ADDR = 0x48;
};

#endif // TMP102_NODE_HPP_