import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, os

sys.path.append(os.path.dirname(__file__))
from vision.detector import deteksi_objek

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub_objek = self.create_publisher(String, '/sensor/vision', 10)
        self.timer = self.create_timer(1.5, self.deteksi)
        self.get_logger().info('✅ Vision Node siap!')

    def deteksi(self):
        objek, confidence = deteksi_objek()
        msg = String()
        if objek is None:
            msg.data = 'NONE:0.0'
            self.get_logger().info('Tidak ada objek terdeteksi')
        else:
            msg.data = f'{objek}:{confidence}'
            if objek == 'person':       level = '🔴 PRIORITAS TINGGI'
            elif objek in ['car','bicycle']: level = '🟡 PERLU DIPERHATIKAN'
            else:                       level = '🟢 NORMAL'
            self.get_logger().info(f'Objek: {objek} | Confidence: {confidence} | {level}')
        self.pub_objek.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
