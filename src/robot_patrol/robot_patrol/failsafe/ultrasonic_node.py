import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import random

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # Publisher jarak ultrasonik
        self.pub_jarak = self.create_publisher(
            Float32, '/sensor/ultrasonik', 10)
        
        # Publisher status bahaya
        self.pub_bahaya = self.create_publisher(
            Bool, '/sensor/ultrasonik_bahaya', 10)
        
        # Timer baca sensor setiap 1 detik
        self.timer = self.create_timer(1.0, self.baca_sensor)
        self.get_logger().info('✅ Ultrasonic Node siap!')

    def baca_sensor(self):
        # Simulasi baca sensor (nanti diganti GPIO asli)
        jarak = round(random.uniform(5.0, 300.0), 2)
        
        # Publish jarak
        msg_jarak = Float32()
        msg_jarak.data = jarak
        self.pub_jarak.publish(msg_jarak)
        
        # Tentukan status bahaya
        bahaya = jarak < 30.0
        msg_bahaya = Bool()
        msg_bahaya.data = bahaya
        self.pub_bahaya.publish(msg_bahaya)
        
        # Log status
        if jarak < 30:
            status = "⚠️  BAHAYA"
        elif jarak < 80:
            status = "⚡ WASPADA"
        else:
            status = "✅ AMAN"
            
        self.get_logger().info(f'Jarak: {jarak} cm | {status}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()