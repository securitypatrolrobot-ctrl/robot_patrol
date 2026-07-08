import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import sys, os

sys.path.append(os.path.dirname(__file__))
from communication.lora_handler import simulasi_kirim_lora, simulasi_terima_lora

class LoraNode(Node):
    def __init__(self):
        super().__init__('lora_node')
        self.pub_status   = self.create_publisher(String, '/comm/lora_status', 10)
        self.pub_perintah = self.create_publisher(String, '/comm/lora_perintah', 10)
        self.sub_nav      = self.create_subscription(String, '/nav/status', self.cb_nav, 10)
        self.nav_status   = 'BERHENTI'
        self.timer = self.create_timer(3.0, self.kirim_data)
        self.get_logger().info('✅ LoRa Node siap!')

    def cb_nav(self, msg):
        self.nav_status = msg.data

    def kirim_data(self):
        data = {'robot_id': 'PATROL-01', 'status': self.nav_status}
        berhasil, pesan = simulasi_kirim_lora(data)
        msg_status = String()
        msg_status.data = 'OK' if berhasil else 'GAGAL'
        self.pub_status.publish(msg_status)
        self.get_logger().info(f'LoRa: {msg_status.data} | {pesan}')
        perintah = simulasi_terima_lora()
        if perintah:
            msg_perintah = String()
            msg_perintah.data = perintah
            self.pub_perintah.publish(msg_perintah)
            self.get_logger().info(f'📩 Perintah: {perintah}')

def main(args=None):
    rclpy.init(args=args)
    node = LoraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
