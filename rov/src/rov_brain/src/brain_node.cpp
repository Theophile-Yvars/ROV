#include "rov_brain/brain_node.hpp"
#include <chrono>
#include <algorithm> // Pour std::max et std::min

using namespace std::chrono_literals;

BrainNode::BrainNode() : Node("brain"), current_temperature_(0.0), cpu_temperature_(0.0), current_depth_(0.0), water_temperature_(0.0), current_pressure_(0.0), cpt_(0) {
    RCLCPP_INFO(this->get_logger(), "Cerveau du ROV démarré (Logic Layer).");

    current_orientation_.x = 0.0;
    current_orientation_.y = 0.0;
    current_orientation_.z = 0.0;

    last_pilot_command_.linear.x = 0.0;
    last_pilot_command_.linear.y = 0.0;
    last_pilot_command_.linear.z = 0.0;
    last_pilot_command_.angular.x = 0.0;
    last_pilot_command_.angular.y = 0.0;
    last_pilot_command_.angular.z = 0.0;

    last_command_time_ = this->now();

    // 1. Abonnements Capteurs
    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>("/rov/temperature", 10, std::bind(&BrainNode::temp_callback, this, std::placeholders::_1));
    cpu_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>("/rov/cpu_temperature", 10, std::bind(&BrainNode::cpu_temp_callback, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>("/rov/water_depth", 10, std::bind(&BrainNode::depth_callback, this, std::placeholders::_1));
    water_temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>("/rov/water_temperature", 10, std::bind(&BrainNode::water_temp_callback, this, std::placeholders::_1));
    pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("/rov/fluid_pressure", 10, std::bind(&BrainNode::pressure_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/rov/orientation", 10, std::bind(&BrainNode::imu_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/rov/cmd_vel_input", 10, std::bind(&BrainNode::cmd_vel_input_callback, this, std::placeholders::_1));
    batt_v_sub_ = this->create_subscription<std_msgs::msg::Float32>("/battery/voltage", 10, std::bind(&BrainNode::battery_voltage_callback, this, std::placeholders::_1));
    camera_tilt_sub_ = this->create_subscription<std_msgs::msg::Float32>("/rov/camera_tilt", 10, std::bind(&BrainNode::camera_tilt_callback, this, std::placeholders::_1));

    // 3. Éditeur vers le nœud de propulsion physique
    motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rov/cmd_vel", 10);
    battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/rov/battery_state", 10);
    latency_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rov/latency", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&BrainNode::control_loop, this));
}

// Callbacks
void BrainNode::temp_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_temperature_ = msg->data; }
void BrainNode::cpu_temp_callback(const std_msgs::msg::Float32::SharedPtr msg) { cpu_temperature_ = msg->data; }
void BrainNode::depth_callback(const std_msgs::msg::Float64::SharedPtr msg) { current_depth_ = msg->data; }
void BrainNode::water_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) { water_temperature_ = msg->temperature; }
void BrainNode::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) { current_pressure_ = msg->fluid_pressure; }
void BrainNode::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { current_orientation_ = *msg; }
void BrainNode::battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) { battery_voltage_ = msg->data; }
void BrainNode::camera_tilt_callback(const std_msgs::msg::Float32::SharedPtr msg) { camera_tilt_value_ = msg->data; }
void BrainNode::cmd_vel_input_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // 1. Calcul de la latence
    if (msg->header.stamp.sec > 0) {
        auto now = this->now();
        auto msg_time = rclcpp::Time(msg->header.stamp);
        double latency_ms = (now - msg_time).seconds() * 1000.0;
        
        if (latency_ms >= 0 && latency_ms < 5000) {
            std_msgs::msg::Float32 lat_msg;
            lat_msg.data = static_cast<float>(latency_ms);
            latency_pub_->publish(lat_msg);
        }
    }

    // 2. Mise à jour de la commande (on copie uniquement le Twist, pas le Stamped)
    last_pilot_command_ = msg->twist;
    last_command_time_ = this->now();

    // 3. Log (Accès via .twist.)
    if (msg->twist.linear.x != 0.0 || msg->twist.linear.z != 0.0 || msg->twist.angular.z != 0.0) {
        RCLCPP_INFO(this->get_logger(), 
            "🕹️ [PILOTE] Ordre reçu -> Avance(X): %.1f | Vertical(Z): %.1f | Pivot(Yaw): %.1f", 
            msg->twist.linear.x, msg->twist.linear.z, msg->twist.angular.z);
    }
}

void BrainNode::control_loop() {
    cpt_++;
    geometry_msgs::msg::Twist safe_command = last_pilot_command_;
    
    // ⏱️ SÉCURITÉ TIMEOUT
    auto age_du_signal = this->now() - last_command_time_;
    if (age_du_signal > rclcpp::Duration(500ms)) {
        safe_command = geometry_msgs::msg::Twist();
        if (cpt_ % 10 == 0) {
            RCLCPP_ERROR(this->get_logger(), "📡 [SÉCURITÉ] Signal perdu ! Arrêt d'urgence.");
        }
    }

    // Sécurité Température
    if (cpu_temperature_ > 75.0) {
        RCLCPP_FATAL(this->get_logger(), "🔥 [SÉCURITÉ] Surchauffe CPU (%.2f °C) !", cpu_temperature_);
        safe_command = geometry_msgs::msg::Twist();
    }

    motor_pub_->publish(safe_command);
    
    // --- AFFICHAGE DE L'ÉTAT DU ROV (1 Hz) ---
    if (cpt_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "--- ÉTAT ROV ---");
        RCLCPP_INFO(this->get_logger(), "Température Interne caisson : %.2f °C", current_temperature_);
        RCLCPP_INFO(this->get_logger(), "Température CPU RPi 5       : %.2f °C", cpu_temperature_); 
        RCLCPP_INFO(this->get_logger(), "Température Externe         : %.2f °C", water_temperature_);
        RCLCPP_INFO(this->get_logger(), "Profondeur Actuelle         : %.2f m", current_depth_);
        RCLCPP_INFO(this->get_logger(), "Orientation IMU             : Cap: %.1f° | Tangage: %.1f° | Roulis: %.1f°", 
                    current_orientation_.z, current_orientation_.y, current_orientation_.x);
        RCLCPP_INFO(this->get_logger(), "Sortie Moteurs (safe_cmd)   : Avance(X): %.1f | Vertical(Z): %.1f | Pivot(Yaw): %.1f",
                    safe_command.linear.x, safe_command.linear.z, safe_command.angular.z);
        if (battery_voltage_ > 0.0 && battery_voltage_ < 10.5) {
            RCLCPP_WARN(this->get_logger(), "⚠️ [ALERTE] Batterie faible : %.2f V", battery_voltage_);
        }else{
            RCLCPP_INFO(this->get_logger(), "Batterie Voltage            : %.2f V", battery_voltage_);
        }
        RCLCPP_INFO(this->get_logger(), "Tilt Caméra                 : %.2f °", camera_tilt_value_);
    }

    // Publication batterie
    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = this->now();
    batt_msg.voltage = battery_voltage_;
    float percentage = ((battery_voltage_ - 10.5f) / (12.6f - 10.5f)) * 100.0f;
    batt_msg.percentage = std::max(0.0f, std::min(100.0f, percentage));
    battery_state_pub_->publish(batt_msg);

    if (cpt_ >= 100) cpt_ = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}