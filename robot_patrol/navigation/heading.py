import rclpy
import serial
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, MagneticField, Imu
from std_msgs.msg import Float32

RAW_TO_TESLA = 3.333e-9
ALPHA        = 0.85   # turun dari 0.95 → kompas lebih dominan, lawan drift gyro

class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sensor_bridge_node')

        self.declare_parameter('port',      '/dev/ttyACM0')
        self.declare_parameter('baudrate',  9600)
        self.declare_parameter('timeout',   1.0)
        self.declare_parameter('frame_gps', 'gps_link')
        self.declare_parameter('frame_mag', 'compass_link')
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('timer_hz',  10.0)

        self.port      = self.get_parameter('port').value
        self.baudrate  = self.get_parameter('baudrate').value
        self.timeout   = self.get_parameter('timeout').value
        self.frame_gps = self.get_parameter('frame_gps').value
        self.frame_mag = self.get_parameter('frame_mag').value
        self.frame_imu = self.get_parameter('frame_imu').value
        timer_hz       = self.get_parameter('timer_hz').value

        # Publishers
        self.pub_gps        = self.create_publisher(NavSatFix,     '/gps/fix',           10)
        self.pub_mag        = self.create_publisher(MagneticField, '/magnetic_field',     10)
        self.pub_heading    = self.create_publisher(Float32,       '/compass/heading',    10)
        self.pub_imu        = self.create_publisher(Imu,           '/imu/data',           10)
        self.pub_heading_cf = self.create_publisher(Float32,       '/compass/heading_cf', 10)

        # CF state
        self.heading_cf  = 0.0
        self.initialized = False

        # Serial state
        self.ser           = None
        self._buffer_ready = False
        self._connect()

        self.create_timer(1.0 / timer_hz, self.read_serial)
        self.get_logger().info(
            "sensor_bridge_node aktif\n"
            "  Topics: /gps/fix | /magnetic_field | /compass/heading | /imu/data | /compass/heading_cf"
        )

    # ── Auto-connect / reconnect ─────────────────────────────
    def _connect(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._buffer_ready = False
                self.get_logger().info(f"Serial terbuka: {self.port} @ {self.baudrate} baud")
                return
            except serial.SerialException as e:
                self.get_logger().warn(f"Gagal buka serial: {e} — coba lagi dalam 2 detik...")
                time.sleep(2.0)

    def _reconnect(self):
        self.get_logger().warn("Serial terputus, mencoba reconnect...")
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._buffer_ready = False
        self._connect()

    # ── Timer callback ───────────────────────────────────────
    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            self._reconnect()
            return

        try:
            if not self.ser.in_waiting:
                return

            # Flush baris tidak lengkap di awal koneksi
            if not self._buffer_ready:
                self.ser.reset_input_buffer()
                self._buffer_ready = True
                return

            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            if raw:
                self.parse_and_publish(raw)

        except serial.SerialException as e:
            self.get_logger().warn(f"Serial error: {e}")
            self._reconnect()
        except OSError as e:
            self.get_logger().warn(f"OS error: {e}")
            self._reconnect()

    # ── Parse & publish ──────────────────────────────────────
    def parse_and_publish(self, line: str):
        if len(line) < 20:
            return
        if not line[0].isdigit() and line[0] != '-':
            return

        parts = line.split(',')
        if len(parts) != 13:
            self.get_logger().warn(f"Data kurang kolom ({len(parts)}/13): '{line}'")
            return

        try:
            lat         = float(parts[0])
            lon         = float(parts[1])
            mx_raw      = float(parts[2])
            my_raw      = float(parts[3])
            mz_raw      = float(parts[4])
            heading_raw = float(parts[5])
            gx_degs     = float(parts[6])
            gy_degs     = float(parts[7])
            gz_degs     = float(parts[8])
            ax_ms2      = float(parts[9])
            ay_ms2      = float(parts[10])
            az_ms2      = float(parts[11])
            dt          = float(parts[12]) / 1000.0  # ms → detik
        except ValueError as e:
            self.get_logger().warn(f"Gagal parse: '{line}' → {e}")
            return

        # Init CF dari kompas pertama kali
        if not self.initialized:
            self.heading_cf  = heading_raw
            self.initialized = True
            self.get_logger().info(f"CF init: heading awal = {self.heading_cf:.1f}°")

        # ── Complementary Filter dengan wraparound handling ──
        # Negatif gz karena konvensi hardware (sesuai cf_node sebelumnya)
        gz_cf = -gz_degs

        # Dead-zone: kalau gyro sangat kecil anggap diam, cegah drift
        if abs(gz_cf) < 0.5:
            gz_cf = 0.0

        # 1. Prediksi dari gyro
        predicted = self.heading_cf + gz_cf * dt

        # 2. Selisih kompas vs prediksi — handle wraparound 0°/360°
        diff = heading_raw - predicted
        if diff >  180: diff -= 360
        if diff < -180: diff += 360

        # 3. Fuse
        self.heading_cf = predicted + (1.0 - ALPHA) * diff

        # 4. Normalisasi 0-360
        self.heading_cf %= 360
        if self.heading_cf < 0:
            self.heading_cf += 360

        now = self.get_clock().now().to_msg()

        self._publish_gps(now, lat, lon)
        self._publish_mag(now, mx_raw, my_raw, mz_raw)
        self._publish_imu(now, gx_degs, gy_degs, gz_degs, ax_ms2, ay_ms2, az_ms2)
        self._publish_heading(heading_raw)
        self._publish_heading_cf(self.heading_cf)

        self.get_logger().debug(
            f"lat={lat:.6f} lon={lon:.6f} | "
            f"hdg_raw={heading_raw:.1f}° hdg_cf={self.heading_cf:.1f}°"
        )

    # ── Publishers ───────────────────────────────────────────
    def _publish_gps(self, stamp, lat, lon):
        msg                          = NavSatFix()
        msg.header.stamp             = stamp
        msg.header.frame_id          = self.frame_gps
        msg.status.status            = NavSatStatus.STATUS_FIX
        msg.status.service           = NavSatStatus.SERVICE_GPS
        msg.latitude                 = lat
        msg.longitude                = lon
        msg.altitude                 = 0.0
        pos_var                      = 2.0
        msg.position_covariance      = [pos_var, 0.0, 0.0,
                                        0.0, pos_var, 0.0,
                                        0.0, 0.0,     4.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_gps.publish(msg)

    def _publish_mag(self, stamp, mx_raw, my_raw, mz_raw):
        msg                              = MagneticField()
        msg.header.stamp                 = stamp
        msg.header.frame_id              = self.frame_mag
        msg.magnetic_field.x             = mx_raw * RAW_TO_TESLA
        msg.magnetic_field.y             = my_raw * RAW_TO_TESLA
        msg.magnetic_field.z             = mz_raw * RAW_TO_TESLA
        msg.magnetic_field_covariance[0] = -1.0
        self.pub_mag.publish(msg)

    def _publish_imu(self, stamp, gx, gy, gz, ax, ay, az):
        msg                       = Imu()
        msg.header.stamp          = stamp
        msg.header.frame_id       = self.frame_imu
        msg.angular_velocity.x    = gx * (math.pi / 180.0)
        msg.angular_velocity.y    = gy * (math.pi / 180.0)
        msg.angular_velocity.z    = gz * (math.pi / 180.0)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        self.pub_imu.publish(msg)

    def _publish_heading(self, heading_deg: float):
        msg      = Float32()
        msg.data = float(heading_deg)
        self.pub_heading.publish(msg)

    def _publish_heading_cf(self, heading_deg: float):
        msg      = Float32()
        msg.data = float(heading_deg)
        self.pub_heading_cf.publish(msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port ditutup.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()