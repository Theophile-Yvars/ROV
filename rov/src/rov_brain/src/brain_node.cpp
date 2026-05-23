#include "rov_brain/brain_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

BrainNode::BrainNode() : Node("brain"), current_temperature_(0.0), cpu_temperature_(0.0), current_depth_(0.0), water_temperature_(0.0), current_pressure_(0.0), cpt_(0) {
    RCLCPP_INFO(this->get_logger(), "Cerveau du ROV démarré.");

    // Initialisation de la structure d'orientation
    current_orientation_.x = 0.0;
    current_orientation_.y = 0.0;
    current_orientation_.z = 0.0;

    // 1. Température interne du caisson (TMP102)
    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/rov/temperature", 10, std::bind(&BrainNode::temp_callback, this, std::placeholders::_1)
    );

    // 2. Température du CPU Raspberry Pi 5
    cpu_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/rov/cpu_temperature", 10, std::bind(&BrainNode::cpu_temp_callback, this, std::placeholders::_1)
    );

    // 3. Profondeur d'eau (MS5837)
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/rov/water_depth", 10, std::bind(&BrainNode::depth_callback, this, std::placeholders::_1)
    );

    // 4. Température de l'eau (MS5837)
    water_temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
        "/rov/water_temperature", 10, std::bind(&BrainNode::water_temp_callback, this, std::placeholders::_1)
    );

    // 5. Pression du fluide (MS5837)
    pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/rov/fluid_pressure", 10, std::bind(&BrainNode::pressure_callback, this, std::placeholders::_1)
    );

    // 6. Centrale Inertielle BNO055
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/rov/orientation", 10, std::bind(&BrainNode::imu_callback, this, std::placeholders::_1)
    );

    // Boucle de contrôle principale (10 Hz)
    timer_ = this->create_wall_timer(100ms, std::bind(&BrainNode::control_loop, this));
}

void BrainNode::temp_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_temperature_ = msg->data;
}

void BrainNode::cpu_temp_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    cpu_temperature_ = msg->data;
}

void BrainNode::depth_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_depth_ = msg->data;
}

void BrainNode::water_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    water_temperature_ = msg->temperature;
}

void BrainNode::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    current_pressure_ = msg->fluid_pressure;
}

void BrainNode::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    current_orientation_ = *msg;
}

void BrainNode::control_loop() {
    cpt_++;
    
    // Affichage mis à jour toutes les secondes
    if (cpt_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "--- ÉTAT ROV ---");
        RCLCPP_INFO(this->get_logger(), "Température Interne caisson : %.2f °C", current_temperature_);
        RCLCPP_INFO(this->get_logger(), "Température CPU RPi 5       : %.2f °C", cpu_temperature_); 
        RCLCPP_INFO(this->get_logger(), "Profondeur Actuelle         : %.2f m", current_depth_);
        RCLCPP_INFO(this->get_logger(), "Température Externe (Eau)  : %.2f °C", water_temperature_);
        RCLCPP_INFO(this->get_logger(), "Pression Fluide            : %.2f Pa", current_pressure_);
        RCLCPP_INFO(this->get_logger(), "Orientation IMU            : Cap: %.1f° | Tangage: %.1f° | Roulis: %.1f°", 
                    current_orientation_.z, current_orientation_.y, current_orientation_.x);
    }

    if (current_depth_ > 10.0) {
        RCLCPP_WARN(this->get_logger(), "Attention : Profondeur critique atteinte (%.2f m) !", current_depth_);
    }
    
    if (cpu_temperature_ > 75.0) {
        RCLCPP_FATAL(this->get_logger(), "Surchauffe critique du CPU de la Raspberry Pi (%.2f °C) !", cpu_temperature_);
    }
    
    if (cpt_ >= 100) {
        cpt_ = 0;
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<BrainNode>();    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}