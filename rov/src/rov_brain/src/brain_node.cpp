#include "rov_brain/brain_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

BrainNode::BrainNode() : Node("brain"), current_temperature_(0.0), cpu_temperature_(0.0), current_depth_(0.0), water_temperature_(0.0), current_pressure_(0.0), cpt_(0) {
    RCLCPP_INFO(this->get_logger(), "Cerveau du ROV démarré.");

    current_orientation_.x = 0.0;
    current_orientation_.y = 0.0;
    current_orientation_.z = 0.0;

    // Initialisation de la commande à zéro
    last_pilot_command_.linear.x = 0.0;
    last_pilot_command_.linear.y = 0.0;
    last_pilot_command_.linear.z = 0.0;
    last_pilot_command_.angular.x = 0.0;
    last_pilot_command_.angular.y = 0.0;
    last_pilot_command_.angular.z = 0.0;

    // ⏱️ Initialisation du timer de sécurité à l'heure actuelle
    last_command_time_ = this->now();

    // 1. Abonnements Capteurs
    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/rov/temperature", 10, std::bind(&BrainNode::temp_callback, this, std::placeholders::_1)
    );
    cpu_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/rov/cpu_temperature", 10, std::bind(&BrainNode::cpu_temp_callback, this, std::placeholders::_1)
    );
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/rov/water_depth", 10, std::bind(&BrainNode::depth_callback, this, std::placeholders::_1)
    );
    water_temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
        "/rov/water_temperature", 10, std::bind(&BrainNode::water_temp_callback, this, std::placeholders::_1)
    );
    pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/rov/fluid_pressure", 10, std::bind(&BrainNode::pressure_callback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/rov/orientation", 10, std::bind(&BrainNode::imu_callback, this, std::placeholders::_1)
    );
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/rov/cmd_vel_input", 10, std::bind(&BrainNode::cmd_vel_input_callback, this, std::placeholders::_1)
    );
    batt_v_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/battery/voltage", 10, std::bind(&BrainNode::battery_voltage_callback, this, std::placeholders::_1)
    );
    batt_i_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/battery/current", 10, std::bind(&BrainNode::battery_current_callback, this, std::placeholders::_1)
    );

    // 3. Éditeur vers le nœud de propulsion physique
    motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rov/cmd_vel", 10);

    // Boucle de contrôle principale (10 Hz)
    timer_ = this->create_wall_timer(100ms, std::bind(&BrainNode::control_loop, this));
}

void BrainNode::temp_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_temperature_ = msg->data; }
void BrainNode::cpu_temp_callback(const std_msgs::msg::Float32::SharedPtr msg) { cpu_temperature_ = msg->data; }
void BrainNode::depth_callback(const std_msgs::msg::Float64::SharedPtr msg) { current_depth_ = msg->data; }
void BrainNode::water_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) { water_temperature_ = msg->temperature; }
void BrainNode::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) { current_pressure_ = msg->fluid_pressure; }
void BrainNode::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { current_orientation_ = *msg; }
void BrainNode::battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) { battery_voltage_ = msg->data; }
void BrainNode::battery_current_callback(const std_msgs::msg::Float32::SharedPtr msg) { battery_current_ = msg->data; }
// Interception des ordres de pilotage
void BrainNode::cmd_vel_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_pilot_command_ = *msg;
    
    // ⏱️ On rafraîchit l'horloge dès qu'un paquet arrive (qu'il soit à 0 ou pas)
    last_command_time_ = this->now();

    if (msg->linear.x != 0.0 || msg->linear.z != 0.0 || msg->angular.z != 0.0) {
        RCLCPP_INFO(this->get_logger(), 
            "🕹️ [PILOTE] Ordre reçu -> Avance(X): %.1f | Vertical(Z): %.1f | Pivot(Yaw): %.1f", 
            msg->linear.x, msg->linear.z, msg->angular.z);
    }
}

void BrainNode::control_loop() {
    cpt_++;

    // --- COUCHE SÉCURITÉ & ARBITRAGE DES MOTEURS ---
    geometry_msgs::msg::Twist safe_command = last_pilot_command_;

    // ⏱️ SÉCURITÉ TIMEOUT : Calcul de l'âge du dernier message reçu
    auto age_du_signal = this->now() - last_command_time_;
    
    if (age_du_signal > rclcpp::Duration(500ms)) {
        // Plus de 500ms sans aucun signe de vie de l'IHM (Liaison coupée, crash du navigateur...)
        safe_command.linear.x = 0.0;
        safe_command.linear.y = 0.0;
        safe_command.linear.z = 0.0;
        safe_command.angular.x = 0.0;
        safe_command.angular.y = 0.0;
        safe_command.angular.z = 0.0;
        
        if (cpt_ % 10 == 0) {
            RCLCPP_ERROR(this->get_logger(), "📡 [SÉCURITÉ] Liaison réseau perdue ! Plus de messages depuis %.1f s. Arrêt d'urgence.", 
                         age_du_signal.seconds());
        }
    }

    // Sécurité 1 : Profondeur
    if (current_depth_ > 10.0) {
        RCLCPP_WARN(this->get_logger(), "Attention : Profondeur critique atteinte (%.2f m) !", current_depth_);
    }
    
    // Sécurité 2 : Surchauffe (Ici safe_command passe à zéro partout)
    if (cpu_temperature_ > 75.0) {
        RCLCPP_FATAL(this->get_logger(), "Surchauffe critique du CPU (%.2f °C) ! Coupure d'urgence des moteurs.", cpu_temperature_);
        safe_command.linear.x = 0.0;
        safe_command.linear.y = 0.0;
        safe_command.linear.z = 0.0;
        safe_command.angular.z = 0.0;
    }

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
        RCLCPP_INFO(this->get_logger(), "Batterie Courant            : %.2f A", battery_current_);
    }

    // Publication finale validée vers la couche hardware
    motor_pub_->publish(safe_command);
    
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