#include "rov_hardware/pression_node.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

PressionNode::PressionNode() : Node("ms5837_node"), i2c_fd_(-1) {
    // Initialisation des tableaux à zéro
    for(int i=0; i<8; ++i) C[i] = 0;

    // Configuration des topics de sortie pour le ROV
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/rov/fluid_pressure", 10);
    depth_pub_    = this->create_publisher<std_msgs::msg::Float64>("/rov/water_depth", 10);
    temp_pub_     = this->create_publisher<sensor_msgs::msg::Temperature>("/rov/water_temperature", 10);

    // Initialisation matérielle du bus I2C (Bus 1 sur Raspberry Pi 5)
    init_i2c();

    // Fréquence d'échantillonnage : 500ms (2 Hz)
    timer_ = this->create_wall_timer(500ms, std::bind(&PressionNode::read_and_publish, this));
}

PressionNode::~PressionNode() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

void PressionNode::init_i2c() {
    const char *device = "/dev/i2c-1";
    
    if ((i2c_fd_ = open(device, O_RDWR)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Impossible d'ouvrir le bus I2C-1");
        return;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, 0x76) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Impossible de contacter le MS5837 à l'adresse 0x76");
        return;
    }

    // Commande de réinitialisation matérielle (Reset)
    uint8_t cmd_reset = 0x1E;
    write(i2c_fd_, &cmd_reset, 1);
    this->get_clock()->sleep_for(rclcpp::Duration(50ms));

    // Lecture des 7 coefficients d'étalonnage en usine stockés dans la PROM
    for (int i = 0; i < 7; i++) {
        uint8_t cmd = 0xA0 + (i * 2);
        uint8_t buffer[2];
        write(i2c_fd_, &cmd, 1);
        read(i2c_fd_, buffer, 2);
        
        // Reconstruction de la valeur sur 16-bits (MSB first)
        C[i] = (buffer[0] << 8) | buffer[1];
    }
    
    RCLCPP_INFO(this->get_logger(), "MS5837 (C++) initialisé ! Calibration chargée.");
}

void PressionNode::read_and_publish() {
    if (i2c_fd_ < 0) return;

    uint8_t buffer[3];
    uint8_t cmd_read = 0x00;
    
    // 1. Commande de conversion de la Pression (D1) - Mode OSR 4096 (Précision max)
    uint8_t cmd_d1 = 0x4A;
    if (write(i2c_fd_, &cmd_d1, 1) != 1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "MS5837: Échec envoi commande D1");
        return;
    }
    this->get_clock()->sleep_for(rclcpp::Duration(20ms)); // Attente de la conversion analogique->numérique
    
    // Lecture de l'ADC pour D1
    if (write(i2c_fd_, &cmd_read, 1) != 1 || read(i2c_fd_, buffer, 3) != 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "MS5837: Échec lecture ADC D1");
        return;
    }
    uint32_t D1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    // 2. Commande de conversion de la Température (D2) - Mode OSR 4096
    uint8_t cmd_d2 = 0x5A;
    if (write(i2c_fd_, &cmd_d2, 1) != 1) {
        return;
    }
    this->get_clock()->sleep_for(rclcpp::Duration(20ms));
    
    // Lecture de l'ADC pour D2
    if (write(i2c_fd_, &cmd_read, 1) != 1 || read(i2c_fd_, buffer, 3) != 3) {
        return;
    }
    uint32_t D2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    // --- FORMULES MATHÉMATIQUES DE COMPENSATION VALIDES ---
    int32_t dT = D2 - (static_cast<int32_t>(C[5]) << 8);
    int32_t TEMP = 2000 + ((static_cast<int64_t>(dT) * C[6]) >> 23);

    int64_t OFF = (static_cast<int64_t>(C[2]) << 16) + ((static_cast<int64_t>(C[4]) * dT) >> 7);
    int64_t SENS = (static_cast<int64_t>(C[1]) << 15) + ((static_cast<int64_t>(C[3]) * dT) >> 8);

    // Pression finale compensée en mbar * 10
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 13; 

    // Conversion dans les unités standards de mesure
    double temp_final = TEMP / 100.0;
    double pressure_bar = P / 10000.0;
    double pressure_pa = P * 10.0; // Unité SI : Pascal

    // Calcul de profondeur basé sur la formule du nœud
    double depth = (pressure_bar - 1.013) * 10.197;
    if (depth < 0.0) depth = 0.0; 

    // --- SÉRIALISATION ET PUBLICATION ---
    auto pr_msg = sensor_msgs::msg::FluidPressure();
    pr_msg.header.stamp = this->now();
    pr_msg.header.frame_id = "pressure_sensor_link";
    pr_msg.fluid_pressure = pressure_pa;
    pressure_pub_->publish(pr_msg);

    auto dp_msg = std_msgs::msg::Float64();
    dp_msg.data = depth;
    depth_pub_->publish(dp_msg);

    auto t_msg = sensor_msgs::msg::Temperature();
    t_msg.header.stamp = this->now();
    t_msg.header.frame_id = "pressure_sensor_link";
    t_msg.temperature = temp_final;
    temp_pub_->publish(t_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressionNode>());
    rclcpp::shutdown();
    return 0;
}