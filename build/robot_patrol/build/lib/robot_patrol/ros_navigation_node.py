import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sys, os, random

sys.path.append(os.path.dirname(__file__))
from navigation.navigation_node import NavigasiRobot

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.pub_status = self.create_publisher(String, '/nav/status', 10)
        self.sub_failsafe = self.create_subscription(Bool, '/failsafe/status', self.cb_failsafe, 10)
        self.sub_perintah = self.create_subscription(String, '/comm/lora_perintah', self.cb_perintah, 10)
        self.failsafe_aktif = False
        self.robot = NavigasiRobot()
        self.timer = self.create_timer(1.5, self.update_navigasi)
        self.get_logger().info('✅ Navigation Node siap!')

    def cb_failsafe(self, msg):
        self.failsafe_aktif = msg.data

    def cb_perintah(self, msg):
        self.robot.terima_perintah(msg.data)
        self.get_logger().info(f'📩 Perintah: {msg.data}')

    def update_navigasi(self):
        if self.failsafe_aktif:
            self.robot.terima_perintah('BERHENTI')
        else:
            self.robot.terima_perintah(random.choice(['MAJU','BELOK_KIRI','BELOK_KANAN']))
        self.robot.update_posisi()
        msg = String()
        msg.data = self.robot.status_sekarang
        self.pub_status.publish(msg)
        self.get_logger().info(f'{self.robot.status_sekarang} | X={self.robot.posisi_x:.1f} Y={self.robot.posisi_y:.1f} Heading={self.robot.heading}°')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
