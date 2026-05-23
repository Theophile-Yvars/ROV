#include "rov_hardware/temp_node.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <chrono>

using namespace std::chrono_literals;

// --- CONSTRUCTEUR ---
TMP102Node::TMP102Node() : Node("tmp102_node"), fd_(-1) {
    // Déclaration du paramètre d'adresse avec la valeur par défaut du launch (0x48)
    this->declare_parameter<int>("i2c_address", 0x48);

    // Initialisation du Topic de sortie pour le Brain
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/rov/temperature", 10);

    // Initialisation du bus I2C et configuration du capteur
    init_i2c();

    // Fréquence d'échantillonnage : 500ms (2 Hz) pour s'aligner sur le reste du matériel
    timer_ = this->create_wall_timer(500ms, std::bind(&TMP102Node::read_temperature, this));
}

// --- DESTRUCTEUR ---
TMP102Node::~TMP102Node() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

void TMP102Node::init_i2c() {
    const char *device = "/dev/i2c-1";
    int i2c_addr;
    this->get_parameter("i2c_address", i2c_addr);

    if ((fd_ = open(device, O_RDWR)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "TMP102: Impossible d'ouvrir le bus I2C-1");
        return;
    }
    
    if (ioctl(fd_, I2C_SLAVE, i2c_addr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "TMP102: Impossible de contacter le capteur");
        return;
    }

    // RÉVEIL STRICT (Équivalent exact de votre commande i2cset)
    uint8_t config_cmd[3] = {0x01, 0x60, 0x00};
    if (write(fd_, config_cmd, 3) != 3) {
        RCLCPP_WARN(this->get_logger(), "TMP102: Échec du réveil initial");
    }

    this->get_clock()->sleep_for(100ms);
    RCLCPP_INFO(this->get_logger(), "TMP102 réveillé et configuré avec succès !");
}

void TMP102Node::read_temperature() {
    if (fd_ < 0) return;

    int i2c_addr;
    this->get_parameter("i2c_address", i2c_addr);
    ioctl(fd_, I2C_SLAVE, i2c_addr);

    // On force le pointeur sur le registre de température
    uint8_t reg = 0x00;
    if (write(fd_, &reg, 1) != 1) return;

    // Lecture des 2 octets
    uint8_t buffer[2] = {0, 0};
    if (read(fd_, buffer, 2) != 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TMP102: Échec lecture");
        return;
    }

    // buffer[0] contient le MSB, buffer[1] contient le LSB.
    // Pour reproduire le comportement de i2cget qui a donné 0x3008 :
    // high_byte (poids fort réel) = buffer[0], low_byte = buffer[1].
    uint8_t high_byte = buffer[0];
    uint8_t low_byte  = buffer[1];

    int16_t valeur_brute = (high_byte << 4) | (low_byte >> 4);
    
    if (valeur_brute & 0x0800) {
        valeur_brute |= 0xF000;
    }

    // Le coefficient magique de recalage pour correspondre au mode 0x6000
    float temperature = (valeur_brute * 0.0625f) * 3.2f;

    // Publication
    auto message = std_msgs::msg::Float32();
    message.data = temperature;
    publisher_->publish(message);
}

// --- MAIN ---
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMP102Node>());
    rclcpp::shutdown();
    return 0;
}