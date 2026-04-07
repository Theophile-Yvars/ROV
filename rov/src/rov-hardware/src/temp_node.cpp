#include "rov_hardware/temp_node.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

TMP102Node::TMP102Node() : Node("tmp102_node"), fd_(-1) {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("rov/temperature", 10);
    
    init_i2c();

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&TMP102Node::read_temperature, this)
    );
}

TMP102Node::~TMP102Node() {
    if (fd_ >= 0) close(fd_);
}

void TMP102Node::init_i2c() {
    const char *device = "/dev/i2c-1";
    if ((fd_ = open(device, O_RDWR)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Erreur : Impossible d'ouvrir /dev/i2c-1");
        return;
    }
    if (ioctl(fd_, I2C_SLAVE, I2C_ADDR) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Erreur : Capteur TMP102 introuvable à 0x48");
    }
}

void TMP102Node::read_temperature() {
    uint8_t reg = 0x00;
    uint8_t data[2];

    if (write(fd_, &reg, 1) != 1 || read(fd_, data, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Échec de lecture sur le bus I2C");
        return;
    }

    // Conversion 12 bits (Format TMP102 classique)
    int16_t res = (data[0] << 4) | (data[1] >> 4);
    if (res > 0x7FF) res |= 0xF000; // Extension de signe pour températures négatives

    auto message = std_msgs::msg::Float32();
    message.data = res * 0.0625f;
    publisher_->publish(message);
}

// Main pour lancer le node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMP102Node>());
    rclcpp::shutdown();
    return 0;
}