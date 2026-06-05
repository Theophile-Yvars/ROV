#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        
        # Initialisation I2C et ADS1115
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c, address=0x49)
        
        # Gain 16 est idéal pour les faibles tensions de shunt (0.256V max)
        self.ads.gain = 16 
        
        self.chan_v = AnalogIn(self.ads, 1) # A1 = Tension
        self.chan_i = AnalogIn(self.ads, 0) # A0 = Intensité
        
        # Configuration
        self.voltage_ratio = 11.2
        self.current_sensitivity = 0.037
        self.current_offset = 0.20
        
        # Variables pour le filtrage (lissage des valeurs)
        self.last_v = 0.0
        self.last_i = 0.0
        
        # Publishers
        self.v_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        self.i_pub = self.create_publisher(Float32, '/battery/current', 10)
        
        # Timer (lecture toutes les 200ms)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Battery Node démarré avec gain 16.")

    def timer_callback(self):
        try:
            # Lecture raw
            v_raw = self.chan_v.voltage
            i_raw = self.chan_i.voltage
            
            # Calculs
            v_instant = v_raw * self.voltage_ratio
            i_instant = max(0, (i_raw / self.current_sensitivity) - self.current_offset)
            
            # Application d'un filtre passe-bas (Lissage)
            # 20% nouvelle valeur, 80% ancienne valeur
            self.last_v = (0.8 * self.last_v) + (0.2 * v_instant)
            self.last_i = (0.8 * self.last_i) + (0.2 * i_instant)
            
            # Publication
            self.v_pub.publish(Float32(data=float(self.last_v)))
            self.i_pub.publish(Float32(data=float(self.last_i)))
            
        except (OSError, RuntimeError):
            # En cas de bruit I2C moteur, on ne fait rien pour laisser
            # les anciennes valeurs publiées (stabilité du dashboard)
            self.get_logger().warn(f"Erreur lecture ADC: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()