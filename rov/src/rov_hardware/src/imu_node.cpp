#include "rov_hardware/imu_node.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <chrono>

using namespace std::chrono_literals;

IMUNode::IMUNode() : Node("imu_node"), fd_(-1) {
    // Adresse par défaut du BNO055 : 0x29
    this->declare_parameter<int>("i2c_address", 0x29);
    this->get_parameter("i2c_address", i2c_addr_);

    // Publication sur un topic d'orientation
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/rov/orientation", 10);

    init_bno055();

    // Lecture rapide (20 Hz soit toutes les 50ms) pour stabiliser le ROV
    timer_ = this->create_wall_timer(
        50ms, 
        std::bind(&IMUNode::read_orientation, this)
    );
}

IMUNode::~IMUNode() {
    if (fd_ >= 0) close(fd_);
}

void IMUNode::init_bno055() {
    const char *device = "/dev/i2c-1";
    if ((fd_ = open(device, O_RDWR)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "IMU : Impossible d'ouvrir /dev/i2c-1");
        return;
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
        RCLCPP_FATAL(this->get_logger(), "IMU : BNO055 introuvable à l'adresse 0x%02X", i2c_addr_);
        close(fd_);
        fd_ = -1;
        return;
    }

    // Configuration de base du BNO055 (Passage en mode NDOF - Fusion complète des capteurs)
    // Registre OPR_MODE (0x3D) <- Valeur 0x0C (NDOF)
    uint8_t config[2] = {0x3D, 0x0C};
    if (write(fd_, config, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "IMU : Échec de configuration du mode NDOF");
    } else {
        RCLCPP_INFO(this->get_logger(), "BNO055 (IMU) initialisé avec succès à l'adresse 0x%02X en mode NDOF.", i2c_addr_);
    }
}

void IMUNode::read_orientation() {
    if (fd_ < 0) return;

    // Registre de départ des données d'Euler : 0x1A
    uint8_t reg = 0x1A;
    uint8_t data[6] = {0};

    if (write(fd_, &reg, 1) != 1 || read(fd_, data, 6) != 6) {
        RCLCPP_ERROR(this->get_logger(), "IMU : Échec de lecture des angles d'Euler");
        return;
    }

    // Le BNO055 renvoie des paires d'octets (LSB, MSB) signées
    // 1 degré = 16 LSB. Il faut donc diviser par 16.0 pour obtenir des degrés.
    int16_t heading_raw = (data[1] << 8) | data[0];
    int16_t roll_raw    = (data[3] << 8) | data[2];
    int16_t pitch_raw   = (data[5] << 8) | data[4];

    auto message = geometry_msgs::msg::Vector3();
    message.x = static_cast<float>(roll_raw) / 16.0f;    // Roulis (Roll)
    message.y = static_cast<float>(pitch_raw) / 16.0f;   // Tangage (Pitch)
    message.z = static_cast<float>(heading_raw) / 16.0f; // Cap / lacet (Heading / Yaw)

    publisher_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}