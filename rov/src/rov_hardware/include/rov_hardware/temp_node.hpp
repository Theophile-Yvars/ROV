#ifndef TMP102_NODE_HPP_
#define TMP102_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class TMP102Node : public rclcpp::Node {
public:
    TMP102Node();
    ~TMP102Node();

private:
    void init_i2c();
    void read_temperature();

    // Membres ROS 2
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Descripteur de fichier pour la liaison I2C Linux (/dev/i2c-1)
    int fd_;
};

#endif // TMP102_NODE_HPP_