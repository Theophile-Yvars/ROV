#include "rov_brain/brain_node.hpp"

BrainNode::BrainNode() : Node("brain_node"){
    RCLCPP_INFO(this->get_logger(), "Brain node started");
    run();
}

void BrainNode::run(){
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        RCLCPP_INFO(this->get_logger(), "Brain node running");
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BrainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}