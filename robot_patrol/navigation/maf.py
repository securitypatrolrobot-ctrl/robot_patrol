import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from collections import deque
import math

class GpsMovingAverageNode(Node):
    def __init__(self):
        super().__init__('gps_moving_average_node')

        # --- Parameter ---
        self.declare_parameter('window_size', 10)
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        
        # --- Parameter Baru: Threshold Gerak (Meter) ---
        # Data baru hanya diterima jika jaraknya > 0.5 meter dari posisi terakhir
        self.declare_parameter('motion_threshold', 0.5) 
        self.motion_threshold = self.get_parameter('motion_threshold').get_parameter_value().double_value

        self.get_logger().info(f'GPS Filter Started. Window: {self.window_size}, Threshold: {self.motion_threshold}m')

        self.lat_buffer = deque(maxlen=self.window_size)
        self.lon_buffer = deque(maxlen=self.window_size)
        self.alt_buffer = deque(maxlen=self.window_size)

        # Simpan posisi terakhir yang valid untuk perbandingan jarak
        self.last_valid_lat = None
        self.last_valid_lon = None

        self.subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/filtered', 10)

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Menghitung jarak antar dua titik GPS dalam meter"""
        R = 6371000  # Radius bumi (meter)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    def listener_callback(self, msg):
        # Abaikan data 0.0 (biasanya data sampah saat baru nyala)
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        # --- LOGIKA FILTER JARAK (Motion Threshold) ---
        should_update = False
        
        if self.last_valid_lat is None:
            # Ini data pertama kali, langsung terima
            should_update = True
        else:
            # Hitung jarak dari posisi terakhir yang disimpan
            dist = self.haversine_distance(
                self.last_valid_lat, self.last_valid_lon,
                msg.latitude, msg.longitude
            )
            
            # Jika jarak > threshold, berarti robot beneran bergerak (atau noise besar)
            if dist > self.motion_threshold:
                should_update = True
            # Jika dist < threshold, anggap robot diam (noise static drift), ABAIKAN.

        if should_update:
            # Masukkan ke Buffer
            self.lat_buffer.append(msg.latitude)
            self.lon_buffer.append(msg.longitude)
            self.alt_buffer.append(msg.altitude)
            
            # Update posisi terakhir yang valid
            self.last_valid_lat = msg.latitude
            self.last_valid_lon = msg.longitude

            # Hitung Rata-rata
            avg_lat = sum(self.lat_buffer) / len(self.lat_buffer)
            avg_lon = sum(self.lon_buffer) / len(self.lon_buffer)
            avg_alt = sum(self.alt_buffer) / len(self.alt_buffer)

            # Publish
            filtered_msg = NavSatFix()
            filtered_msg.header = msg.header
            filtered_msg.status = msg.status
            filtered_msg.latitude = avg_lat
            filtered_msg.longitude = avg_lon
            filtered_msg.altitude = avg_alt
            filtered_msg.position_covariance = msg.position_covariance
            filtered_msg.position_covariance_type = msg.position_covariance_type

            self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsMovingAverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()