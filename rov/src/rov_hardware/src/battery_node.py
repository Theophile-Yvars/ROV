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
        
        # Initialisation du bus I2C avec gestion d'erreur initiale
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.ads = ADS.ADS1115(self.i2c, address=0x49)
        except Exception as e:
            self.get_logger().error(f"Impossible d'initier I2C: {e}")
            raise e
        
        self.voltage_ratio = 11.2
        self.current_sensitivity = 0.037
        self.current_offset = 0.0
        
        self.last_v = 12.0 # Initialisation à 12V pour éviter l'alerte immédiate
        self.last_i = 0.0
        
        self.v_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        #self.i_pub = self.create_publisher(Float32, '/battery/current', 10)
        
        # Timer à 2s pour laisser le bus I2C totalement tranquille
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Battery Node démarré (Mode Ultra-Stab).")

    def timer_callback(self):
        try:
            # 1. Lecture Tension (Canal 0)
            self.ads.gain = 1
            v_raw = AnalogIn(self.ads, 0).voltage
            v_instant = v_raw * self.voltage_ratio
            
            # 2. Lecture Intensité (Canal 1)
            # ON CHANGE LE GAIN : 1 est suffisant pour ne plus saturer
            self.ads.gain = 1
            i_raw = AnalogIn(self.ads, 1).voltage
            
            # ON RETIRE L'OFFSET temporairement pour voir la valeur réelle
            # Une fois que tu auras une valeur cohérente, tu pourras réajuster l'offset
            i_instant = max(0.0, (i_raw / self.current_sensitivity) - self.current_offset)
            
            # Filtre et publication
            self.last_v = (0.9 * self.last_v) + (0.1 * v_instant)
            self.last_i = (0.9 * self.last_i) + (0.1 * i_instant)
            
            self.v_pub.publish(Float32(data=float(self.last_v)))
            #self.i_pub.publish(Float32(data=float(self.last_i)))
            
        except Exception as e:
            # On ne fait rien en cas d'erreur I2C pour ne pas bloquer le nœud
            self.get_logger().warn(f"I2C busy, maintien des anciennes valeurs: {type(e).__name__}")

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