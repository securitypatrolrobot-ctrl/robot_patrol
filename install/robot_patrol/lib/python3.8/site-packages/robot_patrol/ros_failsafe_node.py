import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class FailsafeNode(Node):
    def __init__(self):
        super().__init__('failsafe_node')
        self.pub_failsafe = self.create_publisher(Bool, '/failsafe/status', 10)
        self.sub_ultrasonik = self.create_subscription(Float32, '/sensor/ultrasonik', self.cb_ultrasonik, 10)
        self.sub_lidar = self.create_subscription(Bool, '/sensor/lidar_rintangan', self.cb_lidar, 10)
        self.ultrasonik_cm   = 999.0
        self.lidar_rintangan = False
        self.timer = self.create_timer(0.5, self.cek_failsafe)
        self.get_logger().info('✅ Failsafe Node siap!')

    def cb_ultrasonik(self, msg):
        self.ultrasonik_cm = msg.data

    def cb_lidar(self, msg):
        self.lidar_rintangan = msg.data

    def cek_failsafe(self):
        bahaya = self.ultrasonik_cm < 30.0 or self.lidar_rintangan
        msg = Bool()
        msg.data = bahaya
        self.pub_failsafe.publish(msg)
        if bahaya:
            self.get_logger().error('🚨 FAILSAFE AKTIF - ROBOT BERHENTI!')
        else:
            self.get_logger().info('✅ Sistem aman')

def main(args=None):
    rclpy.init(args=args)
    node = FailsafeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
