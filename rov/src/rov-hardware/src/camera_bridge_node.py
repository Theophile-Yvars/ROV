#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_front')
        # On réduit la queue à 2 pour privilégier les images fraîches (moins de latence)
        self.publisher_ = self.create_publisher(Image, 'image_raw', 2)
        self.bridge = CvBridge()
        
        self.get_logger().info("🔍 Connexion au flux V4L2 (/dev/video10)...")
        
        # --- CONFIGURATION RÉSOLUTION ---
        self.width = 1280
        self.height = 720
        
        self.cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
        
        # Configuration directe du driver
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("❌ ÉCHEC : Impossible d'ouvrir le flux vidéo.")
            return

        self.get_logger().info(f"🚀 Bridge Caméra {self.width}x{self.height} opérationnel !")
        
        # Création du timer à 30 FPS
        self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        # grab() + retrieve() est parfois plus rapide que read() sur les flux V4L2
        if not self.cap.grab():
            return
            
        ret, frame = self.cap.retrieve()
        
        if ret and frame is not None:
            try:
                # On garde bgr8 car c'est le standard ROS pour bcp de nodes
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_optical_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Erreur conversion/envoi : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()