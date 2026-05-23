#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp" // Pratique pour envoyer 3 floats (X, Y, Z / Roll, Pitch, Yaw)

class IMUNode : public rclcpp::Node {
public:
    IMUNode();
    ~IMUNode();

private:
    void init_bno055();
    void read_orientation();

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int fd_;
    int i2c_addr_;
};

#endif // IMU_NODE_HPP_