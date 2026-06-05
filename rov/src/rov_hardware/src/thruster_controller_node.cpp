#include "rov_hardware/thruster_controller_node.hpp"
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <sys/stat.h> // Pour la fonction access

ThrusterControllerNode::ThrusterControllerNode() : Node("thruster_controller_node") {
    RCLCPP_INFO(this->get_logger(), "Initialisation du contrôleur Sysfs pour 4 propulseurs...");

    // Initialisation des 4 canaux PWM du Pi
    for (int id : {MTR_HORIZ_G, MTR_HORIZ_D, MTR_VERT_AV, MTR_VERT_AR}) {
        initSysfsPwm(id);
    }

    RCLCPP_INFO(this->get_logger(), "Armement des 4 ESC (Attente de 3 secondes)...");
    armEscs();
    RCLCPP_INFO(this->get_logger(), "Les 4 ESC sont opérationnels !");

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/rov/cmd_vel", 10, 
        std::bind(&ThrusterControllerNode::cmdVelCallback, this, std::placeholders::_1)
    );
}

ThrusterControllerNode::~ThrusterControllerNode() {
    stopAllMotors();
    RCLCPP_INFO(this->get_logger(), "Fermeture du contrôleur de moteurs.");
}

void ThrusterControllerNode::initSysfsPwm(int pwm_id) {
    // 1. Vérification et Export
    std::string pwm_dir = pwm_path_ + "pwm" + std::to_string(pwm_id);
    struct stat st;
    
    // Si le dossier n'existe pas, on exporte
    if (stat(pwm_dir.c_str(), &st) == -1) {
        std::ofstream export_file(pwm_path_ + "export");
        if (export_file.is_open()) {
            export_file << pwm_id;
            export_file.close();
            usleep(100000); // Temps pour que le noyau crée les fichiers
        }
    }

    // 2. Configuration de la période (50Hz = 20ms)
    std::ofstream period_file(pwm_dir + "/period");
    if (period_file.is_open()) {
        period_file << PWM_PERIOD_NS;
        period_file.close();
    }

    // 3. Initialisation du neutre (1500µs) avant activation
    std::ofstream duty_file(pwm_dir + "/duty_cycle");
    if (duty_file.is_open()) {
        duty_file << (PWM_NEUTRE * 1000);
        duty_file.close();
    }

    // 4. Activation
    std::ofstream enable_file(pwm_dir + "/enable");
    if (enable_file.is_open()) {
        enable_file << "1";
        enable_file.close();
    }
}

void ThrusterControllerNode::setPwmPulseWidth(int pwm_id, int pulse_width_us) {
    long pulse_width_ns = pulse_width_us * 1000;
    std::ofstream duty_file(pwm_path_ + "pwm" + std::to_string(pwm_id) + "/duty_cycle");
    if (duty_file.is_open()) {
        duty_file << pulse_width_ns;
        duty_file.close();
    }
}

void ThrusterControllerNode::armEscs() {
    for (int i = 0; i < 30; ++i) {
        stopAllMotors();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void ThrusterControllerNode::stopAllMotors() {
    setPwmPulseWidth(MTR_HORIZ_G, PWM_NEUTRE);
    setPwmPulseWidth(MTR_HORIZ_D, PWM_NEUTRE);
    setPwmPulseWidth(MTR_VERT_AV,  PWM_NEUTRE);
    setPwmPulseWidth(MTR_VERT_AR,  PWM_NEUTRE);
}

void ThrusterControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double avant_arriere = msg->linear.x;  
    double rotation = msg->angular.z;      
    double monter_descendre = msg->linear.z; 

    // 1. Mixage Moteurs Horizontaux
    double mtr_g_force = avant_arriere + rotation;
    double mtr_d_force = avant_arriere - rotation;

    double max_horizontal = std::max(std::abs(mtr_g_force), std::abs(mtr_d_force));
    if (max_horizontal > 1.0) {
        mtr_g_force /= max_horizontal;
        mtr_d_force /= max_horizontal;
    }

    // 2. Mixage Moteurs Verticaux
    double pwm_g     = PWM_NEUTRE + (mtr_g_force * 400.0);
    double pwm_d     = PWM_NEUTRE + (mtr_d_force * 400.0);
    double pwm_v_av  = PWM_NEUTRE + (monter_descendre * 400.0);
    double pwm_v_ar  = PWM_NEUTRE + (monter_descendre * 400.0);

    // 3. Application avec bornes de sécurité
    setPwmPulseWidth(MTR_HORIZ_G, std::clamp((int)pwm_g, PWM_MAX_ARRIERE, PWM_MAX_AVANT));
    setPwmPulseWidth(MTR_HORIZ_D, std::clamp((int)pwm_d, PWM_MAX_ARRIERE, PWM_MAX_AVANT));
    setPwmPulseWidth(MTR_VERT_AV,  std::clamp((int)pwm_v_av, PWM_MAX_ARRIERE, PWM_MAX_AVANT));
    setPwmPulseWidth(MTR_VERT_AR,  std::clamp((int)pwm_v_ar, PWM_MAX_ARRIERE, PWM_MAX_AVANT));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}