#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_front')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        
        # Ouverture du périphérique loopback
        self.cap = cv2.VideoCapture("/dev/video10", cv2.CAP_V4L2)
        
        # Forcer la résolution HD au niveau d'OpenCV
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.get_logger().info("🚀 Bridge Caméra 1080p démarré")
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # On envoie l'image en BGR8 (Qualité native)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()