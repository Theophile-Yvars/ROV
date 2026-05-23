#ifndef CPU_TEMP_NODE_HPP_
#define CPU_TEMP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class CpuTempNode : public rclcpp::Node {
public:
    CpuTempNode();
    ~CpuTempNode() = default;

private:
    void publish_cpu_temp();

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CPU_TEMP_NODE_HPP_