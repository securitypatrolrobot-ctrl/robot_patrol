import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PatrolPublisher(Node):
    def __init__(self):
        super().__init__('patrol_publisher')
        self.publisher_ = self.create_publisher(String, '/patrol_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot sedang patrol... step ke-{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PatrolPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()