import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys, os

sys.path.append(os.path.dirname(__file__))
from failsafe.lidar_node import baca_lidar, cek_rintangan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.pub_rintangan = self.create_publisher(Bool, '/sensor/lidar_rintangan', 10)
        self.timer = self.create_timer(2.0, self.baca_sensor)
        self.get_logger().info('✅ Lidar Node siap!')

    def baca_sensor(self):
        data = baca_lidar()
        ada_rintangan, jarak = cek_rintangan(data)
        msg = Bool()
        msg.data = ada_rintangan
        self.pub_rintangan.publish(msg)
        if ada_rintangan:
            self.get_logger().info(f'⚠️  RINTANGAN! Jarak: {jarak} m')
        else:
            self.get_logger().info('✅ Jalur depan AMAN')

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
