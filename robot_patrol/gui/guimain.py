#guimain.py

import sys
import csv
import os
import threading
import queue
import time
import math
import traceback
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, MagneticField, Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import serial
import serial.tools.list_ports

try:
    import cv2
    CV2_AVAILABLE = True
except Exception:
    CV2_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    YOLO_AVAILABLE = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QRadioButton, QPushButton, QButtonGroup,
    QComboBox, QScrollArea, QFrame, QSizePolicy, QStackedWidget
)
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygon, QImage, QPixmap
from PyQt5.QtCore import (
    QObject, pyqtSignal, QUrl, QPoint, Qt, QTimer,
    QPropertyAnimation, QRect, QEasingCurve
)
from PyQt5.QtWebEngineWidgets import QWebEngineView


# =========================================================
# Palet warna
# =========================================================
PRIMARY      = "#120078"
BG_APP       = "#E8E8EC"
BG_CARD      = "#FFFFFF"
BORDER       = "#E0E0E8"
WHITE        = "#FFFFFF"
FG_TITLE     = "#FFFFFF"
FG_LABEL     = "#1565C0"
FG_VALUE     = "#111111"
FG_MUTED     = "#888888"
TOPBAR_MUTED = "#C9C3E8"

GREEN_OK  = "#00C853"
RED_ALERT = "#E53935"
BLUE_ACC  = "#1565C0"
COLOR_OK   = "#00C853"
COLOR_BAD  = "#E53935"
COLOR_WARN = "#F57C00"

BTN_BG       = "#3730A3"
BTN_BG_HOVER = "#4C3FC4"

BUMPER_OK_BG  = "#E8F8EE"
BUMPER_BAD_BG = "#FDEAEA"

ALERT_STYLES = {
    "success": ("#E8F8EE", "#00C853"),
    "info":    ("#EAF2FF", "#1565C0"),
    "warn":    ("#FFF6E8", "#F57C00"),
    "danger":  ("#FDEAEA", "#E53935"),
}

# =========================================================
# Konfigurasi serial dashboard 
# =========================================================
BAUD_RATE = 115200
FIELDS_EXPECTED = 20

FIELD_NAMES = [
    "seq", "rssi", "snr", "lat", "lon", "mx", "my", "mz", "heading",
    "dist_cm", "bumper", "gx", "gy", "gz", "ax", "ay", "az", "dt",
    "packetCount"
]

MAX_DIST_CM = 400

# Konversi raw magnetometer -> Tesla (sama kayak readserial.py)
RAW_TO_TESLA = 3.333e-9

DEFAULT_PORT_CANDIDATES = ['/tmp/ttyGPS_gui', '/dev/ttyACM0', '/dev/ttyUSB0']

# Batas jarak buat munculin "Obstacle Warning" di Alerts log (cm)
OBSTACLE_WARN_CM = 40

# Kecepatan gerak manual (D-pad) -> publish ke /cmd_vel
MANUAL_LINEAR_SPEED = 0.2   # m/s
MANUAL_ANGULAR_SPEED = 0.6  # rad/s
MANUAL_PUBLISH_HZ = 10


def haversine_m(lat1, lon1, lat2, lon2):
    """Jarak antara 2 koordinat GPS dalam meter (buat estimasi speed)."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


# =========================================================
# CARD generik: header navy (judul + widget tambahan) + body putih
# =========================================================
class Card(QFrame):
    def __init__(self, title, header_widgets=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame#cardRoot {{
                background:{BG_CARD};
                border-radius:8px;
            }}
        """)
        self.setObjectName("cardRoot")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        header = QWidget()
        header.setStyleSheet(f"background:{PRIMARY}; border-top-left-radius:8px; border-top-right-radius:8px;")
        hl = QHBoxLayout(header)
        hl.setContentsMargins(12, 6, 12, 6)
        hl.setSpacing(8)

        self.title_lbl = QLabel(title)
        self.title_lbl.setStyleSheet("color:white; font-weight:bold; font-size:15px; background:transparent;")
        hl.addWidget(self.title_lbl)
        hl.addStretch(1)

        if header_widgets:
            for w in header_widgets:
                hl.addWidget(w)

        outer.addWidget(header)

        self.body = QWidget()
        self.body.setStyleSheet(f"background:{BG_CARD}; border-bottom-left-radius:8px; border-bottom-right-radius:8px;")
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(10, 10, 10, 10)
        self.body_layout.setSpacing(8)
        outer.addWidget(self.body, 1)


# =========================================================
# STAT CARD kecil (dipakai di Robot status + Location bawah peta)
# =========================================================
class StatCard(QFrame):
    def __init__(self, title, initial="-", parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QFrame {
                background:transparent;
                border:none;
            }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 6)
        layout.setSpacing(4)

        self.title_lbl = QLabel(title)
        self.title_lbl.setAlignment(Qt.AlignCenter)
        self.title_lbl.setStyleSheet(f"color:{FG_LABEL}; font-weight:600; font-size:13px; background:transparent;")
        layout.addWidget(self.title_lbl)

        self.value_lbl = QLabel(initial)
        self.value_lbl.setAlignment(Qt.AlignCenter)
        self.value_lbl.setStyleSheet(f"color:{FG_VALUE}; font-weight:bold; font-size:16px; background:transparent;")
        layout.addWidget(self.value_lbl)

    def set_value(self, text, color=None):
        self.value_lbl.setText(text)
        self.value_lbl.setStyleSheet(
            f"color:{color or FG_VALUE}; font-weight:bold; font-size:16px; background:transparent;"
        )


# =========================================================
# COMPASS WIDGET — cuma dipakai SATU kali, overlay di atas peta
# =========================================================
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.setMinimumSize(120, 140)

    def set_heading(self, heading):
        self.heading = heading
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height() - 18
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 8

        painter.setPen(QPen(QColor("#cccccc"), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255, 235)))
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        for label, angle in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            rad = math.radians(angle)
            lx = cx + int((r - 14) * math.sin(rad))
            ly = cy - int((r - 14) * math.cos(rad))
            color = QColor("#e74c3c") if label == "N" else QColor("#444444")
            painter.setPen(QPen(color))
            painter.drawText(lx - 6, ly + 6, label)

        painter.setPen(QPen(QColor(180, 180, 180, 200), 1))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x1 = cx + int((r - 3) * math.sin(rad))
            y1 = cy - int((r - 3) * math.cos(rad))
            x2 = cx + int((r - 9) * math.sin(rad))
            y2 = cy - int((r - 9) * math.cos(rad))
            painter.drawLine(x1, y1, x2, y2)

        painter.translate(cx, cy)
        painter.rotate(self.heading)

        needle_len = r - 18
        tail_len = r - 45

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#e74c3c")))
        painter.drawPolygon(QPolygon([
            QPoint(0, -needle_len), QPoint(6, 5),
            QPoint(0, 0), QPoint(-6, 5)
        ]))

        painter.setBrush(QBrush(QColor("#aaaaaa")))
        painter.drawPolygon(QPolygon([
            QPoint(0, tail_len), QPoint(5, -3),
            QPoint(0, 0), QPoint(-5, -3)
        ]))

        painter.setBrush(QBrush(QColor("#333333")))
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.drawEllipse(-5, -5, 10, 10)
        painter.resetTransform()

        painter.setPen(QPen(QColor("#222222")))
        painter.drawText(cx - 22, h + 14, f"{self.heading:.1f}°")


# =========================================================
# ALERT ROW — satu baris di Alerts log
# =========================================================
class AlertRow(QFrame):
    def __init__(self, timestamp, title, detail="", kind="info", parent=None):
        super().__init__(parent)
        bg, fg = ALERT_STYLES.get(kind, ALERT_STYLES["info"])
        self.setStyleSheet(f"QFrame {{ background:{bg}; border-radius:5px; }}")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(1)

        top = QHBoxLayout()
        t_lbl = QLabel(timestamp)
        t_lbl.setStyleSheet(f"color:{fg}; font-weight:bold; font-size:10px; background:transparent;")
        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(f"color:{fg}; font-weight:bold; font-size:11px; background:transparent;")
        top.addWidget(t_lbl)
        top.addWidget(title_lbl)
        top.addStretch()
        layout.addLayout(top)

        if detail:
            d_lbl = QLabel(detail)
            d_lbl.setStyleSheet(f"color:#555555; font-size:10px; background:transparent;")
            layout.addWidget(d_lbl)


# =========================================================
# SERIAL READER — baca serial di thread terpisah
# =========================================================
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self._stop_flag = threading.Event()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            self.out_queue.put(("ERROR", f"Gagal buka port {self.port}: {e}"))
            return

        self.out_queue.put(("STATUS", f"Terhubung ke {self.port} @ {self.baud}"))
        while not self._stop_flag.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                self.out_queue.put(("ERROR", f"Serial error: {e}"))
                break
            if line:
                self.out_queue.put(("LINE", line))
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def stop(self):
        self._stop_flag.set()


# =========================================================
# ROS NODE — subscribe topic GPS/compass, publish cmd_vel manual,
# dan self-publish (mode 1 port)
# =========================================================
class GPSNode(Node, QObject):
    data_signal = pyqtSignal(str, float, float, float)
    compass_signal = pyqtSignal(str, float)
    heading_map_signal = pyqtSignal(str, float)

    def __init__(self):
        Node.__init__(self, 'gps_gui_node')
        QObject.__init__(self)

        self._heading_raw = 0.0
        self._heading_cf = 0.0

        self.create_subscription(NavSatFix, '/gps/fix', self.cb_raw, 10)
        self.create_subscription(NavSatFix, '/gps/filtered', self.cb_ekf, 10)
        self.create_subscription(Float32, '/compass/heading', self.cb_heading_raw, 10)
        self.create_subscription(Float32, '/compass/heading_cf', self.cb_heading_cf, 10)

        # Publisher buat mode 1-port (guimain baca serial sendiri, publish
        # sendiri juga -> gantiin peran readserial.py).
        self.pub_gps      = self.create_publisher(NavSatFix,     '/gps/fix',        10)
        self.pub_mag      = self.create_publisher(MagneticField, '/imu/mag',        10)
        self.pub_imu      = self.create_publisher(Imu,           '/imu/data_raw',   10)
        self.pub_heading  = self.create_publisher(Float32,       '/compass/heading', 10)

        # Publisher buat kontrol manual (D-pad panel Manual)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Main GUI node ready (dashboard + map)")

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    # ── Self-publish dari data serial mentah (mode 1 port) ──────
    def publish_from_dashboard(self, lat, lon, heading, mx, my, mz, gx, gy, gz, ax, ay, az):
        now = self.get_clock().now().to_msg()

        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = 'gps_link'
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 0.0
        pos_var = 2.0
        gps_msg.position_covariance = [pos_var, 0.0, 0.0,
                                        0.0, pos_var, 0.0,
                                        0.0, 0.0, 4.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_gps.publish(gps_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = 'compass_link'
        mag_msg.magnetic_field.x = mx * RAW_TO_TESLA
        mag_msg.magnetic_field.y = my * RAW_TO_TESLA
        mag_msg.magnetic_field.z = mz * RAW_TO_TESLA
        mag_msg.magnetic_field_covariance[0] = -1.0
        self.pub_mag.publish(mag_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.x = gx * (math.pi / 180.0)
        imu_msg.angular_velocity.y = gy * (math.pi / 180.0)
        imu_msg.angular_velocity.z = gz * (math.pi / 180.0)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        self.pub_imu.publish(imu_msg)

        hdg_msg = Float32()
        hdg_msg.data = float(heading)
        self.pub_heading.publish(hdg_msg)

    def cb_heading_raw(self, msg):
        self._heading_raw = float(msg.data)
        self.heading_map_signal.emit('raw', self._heading_raw)
        self.compass_signal.emit('raw', self._heading_raw)

    def cb_heading_cf(self, msg):
        self._heading_cf = float(msg.data)
        self.heading_map_signal.emit('ekf', self._heading_cf)
        self.compass_signal.emit('ekf', self._heading_cf)

    def cb_raw(self, msg):
        if msg.latitude != 0.0:
            self.data_signal.emit('raw', msg.latitude, msg.longitude, self._heading_raw)

    def cb_ekf(self, msg):
        if msg.latitude != 0.0:
            self.data_signal.emit('ekf', msg.latitude, msg.longitude, self._heading_cf)


# =========================================================
# Container kecil buat panel peta, biar bisa reposisi compass
# overlay tiap kali di-resize.
# =========================================================
class _MapContainer(QWidget):
    def __init__(self, on_resize, parent=None):
        super().__init__(parent)
        self._on_resize = on_resize

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._on_resize()


# =========================================================
# MAIN WINDOW
# =========================================================
class MainGUIWindow(QMainWindow):
    def __init__(self, node=None):
        super().__init__()
        self.setWindowTitle("Security Patrol Robot Dashboard - RE-054")
        self.resize(1600, 920)
        self._current_mode = 'raw'

        self.node = node
        self.self_publish_ros = False

        self.msg_queue = queue.Queue()
        self.reader_thread = None
        self.got_first_packet = False
        self.auto_reconnect_enabled = True
        self.connect_time = None
        self._last_fix = None  # (lat, lon, monotonic_time) buat estimasi speed

        # Statistik paket dihitung sendiri di GUI (bukan cuma nampilin field
        # mentah dari serial), biar reset tiap konek ulang dan loss-nya
        # ke-detect beneran dari gap nomor seq -- lihat _handle_line().
        self._last_seq = None
        self._loss_total = 0
        self._local_rx_count = 0
        self._prev_bumper = 0
        self._prev_obstacle_zone = None  # None / "ok" / "warn"

                # kamera
        self.cam_index = 6
        self.cap = None
        self.yolo_model = YOLO("yolov8n.pt") if YOLO_AVAILABLE else None

        # manual control
        self._manual_lin = 0.0
        self._manual_ang = 0.0
        self._manual_active = False

        central = QWidget()
        central.setStyleSheet(f"background:{BG_APP};")
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        outer.addWidget(self._build_topbar())

        self.stack = QStackedWidget()
        self.stack.setStyleSheet("background: transparent; border: none;")

        self.pages = {}
        self.pages["Home"] = self._build_home_page()
        self.pages["Data log"] = self._build_datalog_page()
        self.stack.addWidget(self.pages["Home"])
        self.stack.addWidget(self.pages["Data log"])

        outer.addWidget(self.stack, 1)

        self._setup_menu()
        self.show_page("Data log")
        QTimer.singleShot(0, self._rescale_home_page)

        self._refresh_ports()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_queue)
        self.poll_timer.start(50)

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._tick)
        self.clock_timer.start(1000)
        self._tick()

        self._cam_fail_count = 0
        self.cam_auto_reconnect_enabled = True
        if CV2_AVAILABLE:
            self.cam_timer = QTimer(self)
            self.cam_timer.timeout.connect(self._update_camera_frame)
            self.cam_timer.start(33)

            # Auto-connect kamera. Kita SENGAJA nunggu window kebuka dulu
            # (singleShot 800ms) sebelum coba buka v4l2 -- kalau device-nya
            # bermasalah dan bikin native crash yang gak ketangkep
            # try/except Python, minimal window utama udah sempet muncul.
            # Kalau gagal/putus, dicoba lagi tiap beberapa detik otomatis,
            # gak perlu pencet "Connect" manual lagi.
            self.cam_reconnect_timer = QTimer(self)
            self.cam_reconnect_timer.timeout.connect(self._camera_auto_reconnect_tick)
            self.cam_reconnect_timer.start(3000)
            QTimer.singleShot(1500, self._camera_auto_reconnect_tick)

        self.manual_timer = QTimer(self)
        self.manual_timer.timeout.connect(self._publish_manual_cmd)
        self.manual_timer.start(int(1000 / MANUAL_PUBLISH_HZ))

        self._add_alert("System Started", "Robot online, menunggu data GPS ROS...", "success")
        self._add_alert("ROS Mode Active", "Subscribing: /gps/fix", "info")

        # Auto-connect
        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.timeout.connect(self._auto_reconnect_tick)
        self.reconnect_timer.start(2000)
        QTimer.singleShot(300, self._auto_reconnect_tick)

    # ---------------------------------------------------
    # TOP BAR
    # ---------------------------------------------------
    def _build_topbar(self):
        top = QWidget()
        top.setFixedHeight(44)
        top.setStyleSheet(f"background:{PRIMARY};")
        layout = QHBoxLayout(top)
        layout.setContentsMargins(16, 6, 16, 6)
        layout.setSpacing(12)

        menu_lbl = QPushButton("\u2630")
        menu_lbl.setCursor(Qt.PointingHandCursor)
        menu_lbl.setStyleSheet("color:white; font-size:16px; border:none; background:transparent;")
        menu_lbl.clicked.connect(self.toggle_menu)
        layout.addWidget(menu_lbl)

        title = QLabel("Security Patrol Robot")
        title.setStyleSheet(f"color:{FG_TITLE}; font-weight:700; font-size:20px;")
        layout.addWidget(title)

        layout.addStretch(1)

        # Port/refresh/connect balik ditaruh di topbar (compact) -- dipakai
        # buat troubleshoot manual kalau auto-connect-nya belum nyambung.
        self.port_combo = QComboBox()
        self.port_combo.setFixedWidth(170)
        self.port_combo.setStyleSheet("""
            QComboBox {
                background:white; border-radius:12px; padding:4px 10px;
                font-size:11px; color:#222; border:none;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding; subcontrol-position: center right;
                width:22px; border:none; background:transparent;
            }
            QComboBox::down-arrow {
                width:8px; height:8px;
            }
            QComboBox QAbstractItemView {
                border-radius:8px; background:white; selection-background-color:#EEEEF5;
                outline:none;
            }
        """)
        layout.addWidget(self.port_combo)

        self.connect_btn = QPushButton("Disconnected")
        self.connect_btn.setFixedHeight(28)
        self.connect_btn.setStyleSheet(self._conn_btn_style("disconnected"))
        self.connect_btn.clicked.connect(self._toggle_connect)
        layout.addWidget(self.connect_btn)

        btn_export = QPushButton("Export")
        btn_export.setStyleSheet(self._pill_btn_style(bg=WHITE, fg=PRIMARY))
        btn_export.setToolTip("Export alerts log ke file CSV")
        btn_export.clicked.connect(self._export_data)
        layout.addWidget(btn_export)

        self.clock_lbl = QLabel("--:--:--")
        self.clock_lbl.setStyleSheet("color:white; font-weight:bold; font-size:12px;")
        layout.addWidget(self.clock_lbl)

        return top

    def _pill_btn_style(self, bg=None, fg=None):
        bg = bg or BTN_BG
        fg = fg or WHITE
        hover = BTN_BG_HOVER if bg == BTN_BG else "#F0F0F5"
        return f"""
            QPushButton {{
                background:{bg}; color:{fg}; font-weight:bold;
                border-radius:12px; padding:5px 12px; border:none; font-size:11px;
            }}
            QPushButton:hover {{ background:{hover}; }}
        """

    def _conn_btn_style(self, state):
        # state: "connected" / "disconnected" / "connecting" / "error"
        colors = {
            "connected":    ("#0FA958", "#0C8C48"),
            "disconnected": (COLOR_BAD, "#C62828"),
            "connecting":   (COLOR_WARN, "#D97400"),
            "error":        (COLOR_BAD, "#C62828"),
        }
        bg, hover = colors.get(state, colors["disconnected"])
        return f"""
            QPushButton {{
                background:{bg}; color:white; font-weight:bold;
                border-radius:14px; padding:5px 16px; border:none; font-size:11px;
            }}
            QPushButton:hover {{ background:{hover}; }}
        """

    # ---------------------------------------------------
    # PAGES (Home / Data log) + dropdown menu
    # ---------------------------------------------------
    def _build_datalog_page(self):
        """Halaman ini isinya dashboard real-time yang udah ada dari
        awal (peta, kamera, manual, robot status, alerts) -- gak ada
        yang berubah di sumber datanya, cuma dipindah masuk ke page
        "Data log" biar ada halaman Home terpisah."""
        body = QWidget()
        body.setStyleSheet(f"background:{BG_APP};")
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(10, 10, 10, 10)
        body_layout.setSpacing(10)

        row1 = QHBoxLayout()
        row1.setSpacing(10)
        row1.addWidget(self._build_location_card(), 1)
        row1.addWidget(self._build_camera_card(), 1)
        body_layout.addLayout(row1, 3)

        row2 = QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(self._build_manual_card(), 0)
        row2.addWidget(self._build_status_card(), 2)
        row2.addWidget(self._build_alerts_card(), 1)
        body_layout.addLayout(row2, 2)

        return body

    def _build_home_page(self):
        page = QWidget()
        page.setStyleSheet(f"background:{BG_APP};")

        outer = QHBoxLayout(page)
        outer.setContentsMargins(60, 40, 60, 40)
        outer.setSpacing(50)

        # -- poster (opsional, fallback ke placeholder kalau file gak ada) --
        poster_container = QWidget()
        poster_container.setStyleSheet("background:transparent;")
        poster_vlay = QVBoxLayout(poster_container)
        poster_vlay.setContentsMargins(0, 0, 0, 0)
        poster_vlay.addStretch(1)

        poster_lbl = QLabel()
        poster_lbl.setAlignment(Qt.AlignCenter)
        poster_lbl.setStyleSheet("background:transparent;")
        self._home_poster_lbl = poster_lbl

        self._home_poster_orig = QPixmap(os.path.expanduser("~/Downloads/pngpatrol.png"))

        poster_vlay.addWidget(poster_lbl, 0, Qt.AlignCenter)
        poster_vlay.addStretch(1)

        # -- info kanan --
        info_wrap = QWidget()
        info_wrap.setStyleSheet("background:transparent;")
        info = QVBoxLayout(info_wrap)
        info.setSpacing(0)
        info.setContentsMargins(0, 0, 0, 0)

        sub = QLabel("PBL RE-054")
        sub.setStyleSheet(f"color:{FG_MUTED}; font-weight:bold; font-size:14px; background:transparent;")
        self._home_sub = sub

        title_lbl = QLabel("Security Patrol Robot")
        title_lbl.setWordWrap(True)
        title_lbl.setStyleSheet(f"color:{PRIMARY}; font-weight:bold; font-size:30px; background:transparent; margin-top:2px;")
        self._home_title = title_lbl

        desc = QLabel(
            "Robot patroli keamanan outdoor yang dirancang buat bantu tugas "
            "keamanan di berbagai area terbuka. Dilengkapi GPS, kompas, sensor "
            "jarak, bumper, dan kamera live untuk memantau lingkungan sekitar "
            "secara real-time selama patroli."
        )
        desc.setWordWrap(True)
        desc.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        desc.setStyleSheet(f"color:{FG_VALUE}; font-size:13px; background:transparent; margin-top:10px;")
        self._home_desc = desc

        fitur_lbl = QLabel("Fitur")
        fitur_lbl.setStyleSheet(f"color:{PRIMARY}; font-weight:bold; font-size:22px; background:transparent; margin-top:22px;")
        self._home_fitur = fitur_lbl

        icons_row = QHBoxLayout()
        icons_row.setSpacing(32)
        icons_row.setContentsMargins(0, 8, 0, 0)
        icons_row.addWidget(self._home_feat_icon("\U0001F4CD", "Location"))
        icons_row.addWidget(self._home_feat_icon("\U0001F4F7", "Live Cam"))
        icons_row.addWidget(self._home_feat_icon("\U0001F4E1", "Autonomous"))
        icons_row.addStretch()

        info.addStretch(1)
        info.addWidget(sub)
        info.addWidget(title_lbl)
        info.addWidget(desc)
        info.addWidget(fitur_lbl)
        info.addLayout(icons_row)
        info.addStretch(1)

        outer.addWidget(poster_container, 4)
        outer.addWidget(info_wrap, 5)

        return page

    def _home_feat_icon(self, emoji, label):
        w = QWidget()
        w.setStyleSheet("background:transparent;")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        e = QLabel(emoji)
        e.setAlignment(Qt.AlignCenter)
        e.setStyleSheet("font-size:28px; background:transparent;")
        l = QLabel(label)
        l.setAlignment(Qt.AlignCenter)
        l.setStyleSheet(f"color:{FG_LABEL}; font-size:12px; background:transparent;")
        lay.addWidget(e)
        lay.addWidget(l)
        if not hasattr(self, "_home_feat_widgets"):
            self._home_feat_widgets = []
        self._home_feat_widgets.append((e, l))
        return w

    def _setup_menu(self):
        self.menu_drop = QFrame(self)
        self.menu_drop.setStyleSheet(f"background:{WHITE}; border-bottom:1px solid {BORDER};")
        l = QVBoxLayout(self.menu_drop)
        l.setContentsMargins(0, 4, 0, 4)
        l.setSpacing(0)
        for name in ["Home", "Data log"]:
            btn = QPushButton(name)
            btn.setFixedHeight(52)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setStyleSheet(f"""
                QPushButton {{
                    text-align:left; padding-left:24px; border:none;
                    background:transparent; color:{FG_VALUE};
                    font-weight:bold; font-size:16px;
                }}
                QPushButton:hover {{ background:#EFF3FF; color:{PRIMARY}; }}
            """)
            btn.clicked.connect(lambda _checked, n=name: self.switch_page(n))
            l.addWidget(btn)
        self.menu_drop.setGeometry(0, 44, self.width(), 0)
        self._menu_open = False

    def toggle_menu(self):
        target_h = 112 if not self._menu_open else 0
        self._menu_anim = QPropertyAnimation(self.menu_drop, b"geometry")
        self._menu_anim.setDuration(160)
        self._menu_anim.setEasingCurve(QEasingCurve.OutCubic)
        self._menu_anim.setEndValue(QRect(0, 44, self.width(), target_h))
        self._menu_anim.start()
        self._menu_open = not self._menu_open
        self.menu_drop.raise_()

    def switch_page(self, name):
        self.show_page(name)
        if self._menu_open:
            self.toggle_menu()

    def show_page(self, name):
        if name in self.pages:
            self.stack.setCurrentWidget(self.pages[name])
            if name == "Home":
                QTimer.singleShot(0, self._rescale_home_page)

    def _rescale_home_page(self):
        page = self.pages.get("Home")
        if not page:
            return

        # -- poster: scale mengikuti tinggi/lebar yang tersedia --
        if hasattr(self, "_home_poster_orig") and not self._home_poster_orig.isNull():
            orig = self._home_poster_orig
            avail_h = max(300, page.height() - 140)
            h = min(820, avail_h)
            w = int(h * orig.width() / orig.height())
            container = self._home_poster_lbl.parentWidget()
            avail_w = max(200, container.width() - 10) if container else w
            if w > avail_w:
                w = avail_w
                h = int(w * orig.height() / orig.width())
            self._home_poster_lbl.setPixmap(
                orig.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

        # -- teks: font ikut membesar sesuai lebar window --
        scale = max(1.0, min(2.3, page.width() / 1100))
        if hasattr(self, "_home_title"):
            self._home_title.setStyleSheet(
                f"color:{PRIMARY}; font-weight:bold; font-size:{int(30*scale)}px; background:transparent; margin-top:2px;"
            )
        if hasattr(self, "_home_desc"):
            self._home_desc.setStyleSheet(
                f"color:{FG_VALUE}; font-size:{int(13*scale)}px; background:transparent; margin-top:10px;"
            )
        if hasattr(self, "_home_sub"):
            self._home_sub.setStyleSheet(
                f"color:{FG_MUTED}; font-weight:bold; font-size:{int(14*scale)}px; background:transparent;"
            )
        if hasattr(self, "_home_fitur"):
            self._home_fitur.setStyleSheet(
                f"color:{PRIMARY}; font-weight:bold; font-size:{int(22*scale)}px; background:transparent; margin-top:22px;"
            )
        if hasattr(self, "_home_feat_widgets"):
            for e, l in self._home_feat_widgets:
                e.setStyleSheet(f"font-size:{int(28*scale)}px; background:transparent;")
                l.setStyleSheet(f"color:{FG_LABEL}; font-size:{int(12*scale)}px; background:transparent;")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "menu_drop"):
            self.menu_drop.setFixedWidth(self.width())
        QTimer.singleShot(0, self._rescale_home_page)

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._rescale_home_page)
        QTimer.singleShot(150, self._rescale_home_page)

    # ---------------------------------------------------
    # LOCATION CARD (peta + mode buttons + reset + lat/lon)
    # ---------------------------------------------------
    def _build_location_card(self):
        self.rb_raw = QRadioButton("Raw GPS")
        self.rb_raw.setChecked(True)
        self.rb_raw.setStyleSheet("color:#FF8A80; font-weight:bold; font-size:12px;")

        self.rb_ekf = QRadioButton("GPS Filtered")
        self.rb_ekf.setStyleSheet("color:#69F0AE; font-weight:bold; font-size:12px;")

        self.rb_none = QRadioButton("No Tracking")
        self.rb_none.setStyleSheet("color:#82B1FF; font-weight:bold; font-size:12px;")

        group = QButtonGroup(self)
        group.addButton(self.rb_raw)
        group.addButton(self.rb_ekf)
        group.addButton(self.rb_none)

        self.btn_reset = QPushButton("Reset Tracking")
        self.btn_reset.setStyleSheet(self._pill_btn_style(bg=WHITE, fg=RED_ALERT))
        self.btn_reset.clicked.connect(self.reset_path)

        card = Card("\U0001F4CD Location", header_widgets=[
            self.rb_raw, self.rb_ekf, self.rb_none, self.btn_reset
        ])

        map_container = _MapContainer(self._reposition_compass)
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_container.setMinimumHeight(300)

        self.web = QWebEngineView(map_container)
        map_layout.addWidget(self.web)

        self.map_compass = CompassWidget(map_container)
        self.map_compass.setFixedSize(120, 140)
        self.map_compass.raise_()

        self._map_container = map_container
        card.body_layout.addWidget(map_container, 1)

        # baris kecil lat/lon di bawah peta -- satu baris tipis aja,
        # bukan dua kotak StatCard gede kayak sebelumnya
        latlon_row = QHBoxLayout()
        latlon_row.setContentsMargins(4, 4, 4, 2)
        latlon_row.setSpacing(24)

        self.card_lat = QLabel("Lat: -")
        self.card_lat.setStyleSheet(f"color:{FG_VALUE}; font-size:12px; font-weight:600; background:transparent;")
        self.card_lon = QLabel("Long: -")
        self.card_lon.setStyleSheet(f"color:{FG_VALUE}; font-size:12px; font-weight:600; background:transparent;")

        latlon_row.addWidget(self.card_lat)
        latlon_row.addWidget(self.card_lon)
        latlon_row.addStretch()
        card.body_layout.addLayout(latlon_row)

        self.map_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "guimain_map.html"
        )
        self.init_map_html(1.118489, 104.048447)
        self.web.setUrl(QUrl.fromLocalFile(self.map_file))

        self.rb_raw.toggled.connect(lambda checked: self._on_switch("raw") if checked else None)
        self.rb_ekf.toggled.connect(lambda checked: self._on_switch("ekf") if checked else None)
        self.rb_none.toggled.connect(lambda checked: self._on_switch("none") if checked else None)

        return card

    def _reposition_compass(self):
        if hasattr(self, 'map_compass') and hasattr(self, '_map_container'):
            cw = self.map_compass.width()
            ch = self.map_compass.height()
            mw = self._map_container.width()
            mh = self._map_container.height()
            self.map_compass.move(mw - cw - 12, mh - ch - 12)
            self.map_compass.raise_()

    def _on_switch(self, mode):
        self._current_mode = mode
        self.switch(mode)

    def init_map_html(self, lat, lon):
        html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
html, body {{ margin:0; padding:0; }}
#map {{ width:100%; height:100vh; }}
</style>
</head>
<body>
<div id="map"></div>
<script>
var map = L.map('map').setView([{lat}, {lon}], 18);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{ maxZoom:19 }}).addTo(map);

var rawPath = L.polyline([], {{
  color:'red', weight:4, opacity:0.9,
  lineJoin:'round', lineCap:'round'
}}).addTo(map);

var ekfPath = L.polyline([], {{
  color:'green', weight:4, opacity:0.9,
  lineJoin:'round', lineCap:'round'
}}).addTo(map);

function makeArrowIcon(color, heading) {{
  var svg = '<svg xmlns="http://www.w3.org/2000/svg" width="40" height="40" viewBox="0 0 40 40">'
    + '<g transform="rotate(' + heading + ', 20, 20)">'
    + '<polygon points="20,8 27,28 20,24 13,28" fill="' + color + '" stroke="white" stroke-width="1.5"/>'
    + '</g></svg>';
  return L.divIcon({{ html:svg, className:'', iconSize:[40,40], iconAnchor:[20,20] }});
}}

var rawMarker = L.marker([{lat},{lon}], {{ icon: makeArrowIcon('red',   0) }}).addTo(map);
var ekfMarker = L.marker([{lat},{lon}], {{ icon: makeArrowIcon('green', 0) }}).addTo(map);

map.removeLayer(ekfPath);
map.removeLayer(ekfMarker);

function updatePosition(mode, lat, lon, heading) {{
  var p = L.latLng(lat, lon);
  if (mode === 'raw') {{
    rawPath.addLatLng(p);
    rawMarker.setLatLng(p);
    rawMarker.setIcon(makeArrowIcon('red', heading));
  }}
  if (mode === 'ekf') {{
    ekfPath.addLatLng(p);
    ekfMarker.setLatLng(p);
    ekfMarker.setIcon(makeArrowIcon('green', heading));
  }}
  map.setView(p);
}}

function updateHeading(mode, heading) {{
  if (mode === 'raw') {{
    rawMarker.setIcon(makeArrowIcon('red',   heading));
  }} else {{
    ekfMarker.setIcon(makeArrowIcon('green', heading));
  }}
}}

function switchLayer(mode) {{
  if (mode === 'raw') {{
    map.addLayer(rawPath);    map.addLayer(rawMarker);
    map.removeLayer(ekfPath); map.removeLayer(ekfMarker);
  }} else if (mode === 'ekf') {{
    map.addLayer(ekfPath);    map.addLayer(ekfMarker);
    map.removeLayer(rawPath); map.removeLayer(rawMarker);
  }} else {{
    map.removeLayer(rawPath);  map.removeLayer(rawMarker);
    map.removeLayer(ekfPath);  map.removeLayer(ekfMarker);
  }}
}}

function clearPath() {{
  rawPath.setLatLngs([]);
  ekfPath.setLatLngs([]);
}}
</script>
</body>
</html>"""
        with open(self.map_file, "w") as f:
            f.write(html)

    def update_map(self, mode, lat, lon, heading):
        self.web.page().runJavaScript(f"updatePosition('{mode}', {lat}, {lon}, {heading});")

    def update_compass(self, mode, heading):
        if mode == self._current_mode or self._current_mode == 'none':
            self.map_compass.set_heading(heading)

    def update_heading_map(self, mode, heading):
        self.web.page().runJavaScript(f"updateHeading('{mode}', {heading});")

    def switch(self, mode):
        self.web.page().runJavaScript(f"switchLayer('{mode}');")

    def reset_path(self):
        self.web.page().runJavaScript("clearPath();")

    # ---------------------------------------------------
    # LIVE CAMERA CARD
    # ---------------------------------------------------
    def _build_camera_card(self):
        self.cam_connect_btn = QPushButton("Connect")
        self.cam_connect_btn.setStyleSheet(self._pill_btn_style(bg=WHITE, fg=PRIMARY))
        self.cam_connect_btn.clicked.connect(self._toggle_camera)
        if not CV2_AVAILABLE:
            self.cam_connect_btn.setEnabled(False)

        card = Card("\U0001F4F7 Live camera", header_widgets=[self.cam_connect_btn])

        self.cam_view = QLabel()
        self.cam_view.setAlignment(Qt.AlignCenter)
        self.cam_view.setMinimumHeight(300)
        self.cam_view.setStyleSheet("background:#111111; border-radius:6px; color:#888888; font-weight:bold;")
        if not CV2_AVAILABLE:
            self.cam_view.setText("OpenCV TIDAK TERPASANG\n(pip install opencv-python)")
        else:
            self.cam_view.setText(f"NO CAMERA FEED\n(index {self.cam_index} — pencet Connect)")

        card.body_layout.addWidget(self.cam_view, 1)
        return card

    def _toggle_camera(self):
        if not CV2_AVAILABLE:
            return
        if self.cap is not None:
            # User matiin manual -> jangan auto-reconnect lagi sampai
            # dia pencet Connect sendiri.
            self.cam_auto_reconnect_enabled = False
            self._close_camera()
            self.cam_connect_btn.setText("Connect")
            self.cam_view.setText(f"NO CAMERA FEED\n(index {self.cam_index} — pencet Connect)")
            return
        self.cam_auto_reconnect_enabled = True
        self._open_camera(self.cam_index)

    def _camera_auto_reconnect_tick(self):
        if not CV2_AVAILABLE or not self.cam_auto_reconnect_enabled:
            return
        if self.cap is not None:
            return
        self._open_camera(self.cam_index)
        if self.cap is None:
            # Index ini gagal -- coba index berikutnya di percobaan
            # selanjutnya (auto-scan), sama kayak kalau user pencet
            # tombol "\u203A" manual buat cari kamera yang bener.
            self.cam_index = (self.cam_index + 1) % 4

    def _close_camera(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        self._cam_fail_count = 0

    def _open_camera(self, index):
        """Buka kamera dengan hati-hati. Kalau device-nya bermasalah
        (driver v4l error dll), langsung dilepas lagi tanpa nge-crash GUI."""
        if not CV2_AVAILABLE:
            return
        self._close_camera()
        try:
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                self.cam_view.setText(f"NO CAMERA FEED\n(index {index} gagal dibuka)")
                self._add_alert("Kamera", f"Gagal buka kamera index {index}.", "warn")
                return
            # test-read sekali dulu sebelum dipakai beneran, biar ketauan
            # kalau device-nya "ke-detect" tapi sebenernya gak bisa nge-capture
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                self.cam_view.setText(f"NO CAMERA FEED\n(index {index} gak bisa capture)")
                self._add_alert("Kamera", f"Kamera index {index} terdeteksi tapi gak bisa capture frame.", "warn")
                return
            self.cap = cap
            self.cam_connect_btn.setText("Disconnect")
            self._add_alert("Kamera", f"Kamera index {index} terhubung.", "success")
        except Exception:
            print(traceback.format_exc())
            self.cam_view.setText(f"NO CAMERA FEED\n(index {index} error)")
            self.cap = None

    def _cycle_camera(self):
        if not CV2_AVAILABLE:
            return
        was_connected = self.cap is not None
        self.cam_index = (self.cam_index + 1) % 4
        if was_connected:
            self._open_camera(self.cam_index)
        else:
            self.cam_view.setText(f"NO CAMERA FEED\n(index {self.cam_index} — pencet Connect)")

    def _update_camera_frame(self):
        if not CV2_AVAILABLE or self.cap is None:
            return
        try:
            ok, frame = self.cap.read()
        except Exception:
            ok, frame = False, None
        if not ok or frame is None:
            self._cam_fail_count += 1
            if self._cam_fail_count >= 5:
                self._close_camera()
                self.cam_connect_btn.setText("Connect")
                self.cam_view.setText(f"NO CAMERA FEED\n(index {self.cam_index} terputus)")
                self._add_alert("Kamera", "Koneksi kamera terputus.", "danger")
            return
        self._cam_fail_count = 0
        try:
            # --- YOLO detection ---
            if self.yolo_model is not None:
                results = self.yolo_model(frame, verbose=False)
                frame = results[0].plot()

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            target_w = max(1, self.cam_view.width())
            target_h = max(1, self.cam_view.height())
            scaled = QPixmap.fromImage(qimg).scaled(
                target_w, target_h,
                Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation
            )
            x_off = max(0, (scaled.width() - target_w) // 2)
            y_off = max(0, (scaled.height() - target_h) // 2)
            pix = scaled.copy(x_off, y_off, target_w, target_h)
            self.cam_view.setPixmap(pix)
        except Exception:
            print(traceback.format_exc())

    # ---------------------------------------------------
    # MANUAL CARD (D-pad -> /cmd_vel)
    # ---------------------------------------------------
    def _build_manual_card(self):
        card = Card("\u2B07 Manual")
        card.setMinimumWidth(240)
        card.setMaximumWidth(300)

        grid = QGridLayout()
        grid.setSpacing(14)

        def make_dir_btn(text, lin, ang):
            btn = QPushButton(text)
            btn.setFixedSize(74, 74)
            btn.setStyleSheet(f"""
                QPushButton {{ background:#EEEEF5; color:{PRIMARY}; font-weight:bold;
                               border-radius:16px; font-size:22px; }}
                QPushButton:pressed {{ background:{BTN_BG}; color:white; }}
            """)
            btn.pressed.connect(lambda: self._manual_set(lin, ang))
            btn.released.connect(lambda: self._manual_set(0.0, 0.0))
            return btn

        btn_up = make_dir_btn("\u2191", MANUAL_LINEAR_SPEED, 0.0)
        btn_down = make_dir_btn("\u2193", -MANUAL_LINEAR_SPEED, 0.0)
        btn_left = make_dir_btn("\u2190", 0.0, MANUAL_ANGULAR_SPEED)
        btn_right = make_dir_btn("\u2192", 0.0, -MANUAL_ANGULAR_SPEED)

        btn_stop = QPushButton("STOP")
        btn_stop.setFixedSize(74, 74)
        btn_stop.setStyleSheet(f"""
            QPushButton {{ background:{COLOR_BAD}; color:white; font-weight:bold;
                           border-radius:37px; font-size:13px; }}
            QPushButton:hover {{ background:#C62828; }}
        """)
        btn_stop.clicked.connect(lambda: self._manual_set(0.0, 0.0))

        grid.addWidget(btn_up, 0, 1)
        grid.addWidget(btn_left, 1, 0)
        grid.addWidget(btn_stop, 1, 1)
        grid.addWidget(btn_right, 1, 2)
        grid.addWidget(btn_down, 2, 1)

        wrap = QHBoxLayout()
        wrap.addStretch()
        wrap.addLayout(grid)
        wrap.addStretch()
        card.body_layout.addStretch()
        card.body_layout.addLayout(wrap)
        card.body_layout.addStretch()
        return card

    def _manual_set(self, lin, ang):
        self._manual_lin = lin
        self._manual_ang = ang
        self._manual_active = (lin != 0.0 or ang != 0.0)

    def _publish_manual_cmd(self):
        if self.node is None:
            return
        try:
            self.node.publish_cmd_vel(self._manual_lin, self._manual_ang)
        except Exception:
            print(traceback.format_exc())

    # ---------------------------------------------------
    # ROBOT STATUS CARD
    # ---------------------------------------------------
    def _block_frame(self):
        """Satu 'blok' kartu kecil dengan border + background, dipakai
        buat misahin tiap kelompok data di Robot status biar rapi.
        Putih tegas + border lebih gelap biar kontras jelas kelihatan
        di atas background abu-abu body Robot status."""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background:#FFFFFF; border:1px solid #D5D5E0; border-radius:10px;
            }
        """)
        return frame

    def _build_status_card(self):
        card = Card("\U0001F4CA Robot status")

        # body Robot status dikasih background abu-abu muda biar blok
        # putihnya kontras jelas (bukan putih di atas putih)
        grey_wrap = QWidget()
        grey_wrap.setStyleSheet("background:#EFEFF3; border-radius:8px;")
        grey_lay = QVBoxLayout(grey_wrap)
        grey_lay.setContentsMargins(12, 12, 12, 12)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        grid.setRowStretch(2, 1)

        # -- blok Communication package (kiri, tinggi penuh 3 baris) --
        comm_frame = self._block_frame()
        comm_layout = QVBoxLayout(comm_frame)
        comm_layout.setContentsMargins(10, 10, 10, 10)
        comm_layout.setSpacing(10)
        comm_title = QLabel("Communication package")
        comm_title.setAlignment(Qt.AlignCenter)
        comm_title.setStyleSheet(f"color:{FG_LABEL}; font-weight:bold; font-size:15px; background:transparent; border:none;")

        comm_grid = QGridLayout()
        comm_grid.setHorizontalSpacing(16)
        comm_grid.setVerticalSpacing(12)
        self.card_rssi = self._mini_stat("RSSI")
        self.card_snr = self._mini_stat("SNR")
        self.card_seq = self._mini_stat("SEQ")
        self.card_pktcount = self._mini_stat("ACK")
        self.card_loss = self._mini_stat("Paket hilang")
        comm_grid.addWidget(self.card_rssi, 0, 0)
        comm_grid.addWidget(self.card_snr, 0, 1)
        comm_grid.addWidget(self.card_seq, 1, 0)
        comm_grid.addWidget(self.card_pktcount, 1, 1)
        comm_grid.addWidget(self.card_loss, 2, 0, 1, 2)

        comm_layout.addStretch(1)
        comm_layout.addWidget(comm_title)
        comm_layout.addSpacing(14)
        comm_layout.addLayout(comm_grid)
        comm_layout.addStretch(1)

        grid.addWidget(comm_frame, 0, 0, 3, 1)

        # -- blok Speed --
        speed_frame = self._block_frame()
        speed_lay = QVBoxLayout(speed_frame)
        speed_lay.setContentsMargins(8, 8, 8, 8)
        self.card_speed = StatCard("Speed", "0.00 m/s")
        speed_lay.addStretch(1)
        speed_lay.addWidget(self.card_speed)
        speed_lay.addStretch(1)
        grid.addWidget(speed_frame, 0, 1)

        # -- blok Battery --
        battery_frame = self._block_frame()
        battery_lay = QVBoxLayout(battery_frame)
        battery_lay.setContentsMargins(8, 8, 8, 8)
        self.card_battery = StatCard("Battery", "N/A")
        battery_lay.addStretch(1)
        battery_lay.addWidget(self.card_battery)
        battery_lay.addStretch(1)
        grid.addWidget(battery_frame, 1, 1)

        # -- blok Ultrasonic --
        dist_frame = self._block_frame()
        dist_lay = QVBoxLayout(dist_frame)
        dist_lay.setContentsMargins(8, 8, 8, 8)
        self.card_dist = StatCard("Ultrasonic", "-- cm")
        dist_lay.addStretch(1)
        dist_lay.addWidget(self.card_dist)
        dist_lay.addStretch(1)
        grid.addWidget(dist_frame, 0, 2)

        # -- blok Uptime --
        uptime_frame = self._block_frame()
        uptime_lay = QVBoxLayout(uptime_frame)
        uptime_lay.setContentsMargins(8, 8, 8, 8)
        self.card_uptime = StatCard("Uptime", "00:00:00")
        uptime_lay.addStretch(1)
        uptime_lay.addWidget(self.card_uptime)
        uptime_lay.addStretch(1)
        grid.addWidget(uptime_frame, 1, 2)

        # -- blok Bumper (tinggi 2 baris) --
        bumper_frame = self._block_frame()
        bumper_lay = QVBoxLayout(bumper_frame)
        bumper_lay.setContentsMargins(8, 8, 8, 8)
        self.bumper_indicator = StatCard("Bumper", "AMAN")
        self.bumper_indicator.value_lbl.setStyleSheet(
            f"color:{COLOR_OK}; font-weight:bold; font-size:20px; background:transparent;"
        )
        bumper_lay.addStretch(1)
        bumper_lay.addWidget(self.bumper_indicator)
        bumper_lay.addStretch(1)
        grid.addWidget(bumper_frame, 0, 3, 2, 1)

        # -- blok IMU (accel/gyro), lebar penuh baris bawah --
        imu_frame = self._block_frame()
        imu_layout = QVBoxLayout(imu_frame)
        imu_layout.setContentsMargins(10, 10, 10, 10)
        imu_layout.setSpacing(8)
        imu_title = QLabel("IMU")
        imu_title.setAlignment(Qt.AlignCenter)
        imu_title.setStyleSheet(f"color:{FG_LABEL}; font-weight:bold; font-size:15px; background:transparent; border:none;")
        self.card_accel = self._mini_stat("Accel X/Y/Z (m/s^2)")
        self.card_gyro = self._mini_stat("Gyro X/Y/Z (deg/s)")

        imu_row = QHBoxLayout()
        imu_row.setSpacing(20)
        imu_row.addWidget(self.card_accel)
        imu_row.addWidget(self.card_gyro)

        imu_layout.addStretch(1)
        imu_layout.addWidget(imu_title)
        imu_layout.addSpacing(14)
        imu_layout.addLayout(imu_row)
        imu_layout.addStretch(1)
        grid.addWidget(imu_frame, 2, 1, 1, 3)

        grey_lay.addLayout(grid)
        card.body_layout.addWidget(grey_wrap, 1)

        return card

    def _mini_stat(self, title):
        return StatCard(title, "-")

    # ---------------------------------------------------
    # ALERTS LOG CARD
    # ---------------------------------------------------
    def _build_alerts_card(self):
        btn_clear = QPushButton("Clear All")
        btn_clear.setStyleSheet(self._pill_btn_style(bg=WHITE, fg=PRIMARY))
        btn_clear.clicked.connect(self._clear_alerts)

        card = Card("\U0001F514 Alerts log", header_widgets=[btn_clear])

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"""
            QScrollArea {{ border:none; background:transparent; }}
            QScrollBar:vertical {{
                background:transparent; width:8px; margin:2px;
            }}
            QScrollBar::handle:vertical {{
                background:#C7C7D6; border-radius:4px; min-height:24px;
            }}
            QScrollBar::handle:vertical:hover {{ background:{BTN_BG}; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height:0px; background:transparent;
            }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
                background:transparent;
            }}
        """)

        self.alerts_body = QWidget()
        self.alerts_layout = QVBoxLayout(self.alerts_body)
        self.alerts_layout.setSpacing(6)
        self.alerts_layout.addStretch()
        scroll.setWidget(self.alerts_body)

        card.body_layout.addWidget(scroll, 1)
        return card

    def _add_alert(self, title, detail="", kind="info"):
        ts = time.strftime("%H:%M:%S")
        row = AlertRow(ts, title, detail, kind)
        # baru selalu masuk paling atas
        self.alerts_layout.insertWidget(0, row)
        if not hasattr(self, "alerts_history"):
            self.alerts_history = []
        self.alerts_history.append((ts, title, detail, kind))

    def _clear_alerts(self):
        while self.alerts_layout.count() > 1:
            item = self.alerts_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

    def _export_data(self):
        history = getattr(self, "alerts_history", [])
        if not history:
            self._add_alert("Export", "Belum ada data alerts buat di-export.", "warn")
            return
        out_dir = os.path.expanduser("~/gps_pkg_exports")
        try:
            os.makedirs(out_dir, exist_ok=True)
            fname = f"alerts_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            fpath = os.path.join(out_dir, fname)
            with open(fpath, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "title", "detail", "kind"])
                for row in history:
                    writer.writerow(row)
            self._add_alert("Export", f"Tersimpan: {fpath}", "success")
        except Exception as e:
            self._add_alert("Export", f"Gagal export: {e}", "danger")

    # ---------------------------------------------------
    # CLOCK / UPTIME TICK
    # ---------------------------------------------------
    def _tick(self):
        self.clock_lbl.setText(time.strftime("%H:%M:%S"))
        if self.connect_time is not None:
            elapsed = int(time.time() - self.connect_time)
            hh = elapsed // 3600
            mm = (elapsed % 3600) // 60
            ss = elapsed % 60
            self.card_uptime.set_value(f"{hh:02d}:{mm:02d}:{ss:02d}")

    # ---------------------------------------------------
    # SERIAL — connect / disconnect / polling
    # ---------------------------------------------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        virtual_port = "/tmp/ttyGPS_gui"
        if os.path.exists(virtual_port) and virtual_port not in ports:
            ports.insert(0, virtual_port)
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _pick_default_port(self):
        env_port = os.environ.get('GPS_GUI_PORT')
        if env_port:
            return env_port
        # Selalu prioritaskan virtual port dari splitter kalau ada, biar GUI
        # gak pernah rebutan akses langsung ke port fisik /dev/ttyUSB0.
        virtual_port = '/tmp/ttyGPS_gui'
        if os.path.exists(virtual_port):
            return virtual_port
        for candidate in DEFAULT_PORT_CANDIDATES:
            if os.path.exists(candidate):
                return candidate
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports[0] if ports else None

    def _should_self_publish(self, port):
        return 'ttyGPS' not in port

    def _connect_port(self, port):
        if not port:
            self._set_conn_status("error", "Pilih port dulu")
            return
        if self.reader_thread and self.reader_thread.is_alive():
            return

        self.self_publish_ros = self._should_self_publish(port)
        self.reader_thread = SerialReader(port, BAUD_RATE, self.msg_queue)
        self.reader_thread.start()
        self._last_seq = None
        self._loss_total = 0
        self._local_rx_count = 0
        self._set_conn_status("connecting", "Connecting...")
        mode_txt = "self-publish ke ROS (mode 1 port)" if self.self_publish_ros else "via splitter (readserial yang publish)"
        self._add_alert("Connecting", f"{port} — {mode_txt}", "info")

    def _disconnect(self):
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.stop()
        self.reader_thread = None
        self._set_conn_status("disconnected", "Disconnected")
        self.connect_time = None
        self._add_alert("Disconnected", "", "danger")

    def _toggle_connect(self):
        if self.reader_thread and self.reader_thread.is_alive():
            self.auto_reconnect_enabled = False
            self._disconnect()
            return

        self.auto_reconnect_enabled = True
        port = self.port_combo.currentText() or self._pick_default_port()
        if port and self.port_combo.findText(port) < 0:
            self.port_combo.insertItem(0, port)
        if port:
            self.port_combo.setCurrentText(port)
        self._connect_port(port)

    def _auto_reconnect_tick(self):
        if not self.auto_reconnect_enabled:
            return
        if self.reader_thread and self.reader_thread.is_alive():
            return
        self._refresh_ports()
        port = self._pick_default_port()
        if not port:
            return
        idx = self.port_combo.findText(port)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
        else:
            self.port_combo.insertItem(0, port)
            self.port_combo.setCurrentIndex(0)
        self._connect_port(port)

    def _set_conn_status(self, state, text):
        """state: 'connected' / 'disconnected' / 'connecting' / 'error'.
        Satu tombol ini sekaligus jadi indikator (warna) DAN tombol
        connect/disconnect -- gak perlu label status terpisah lagi."""
        self.connect_btn.setText(text)
        self.connect_btn.setStyleSheet(self._conn_btn_style(state))

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.msg_queue.get_nowait()
                if kind == "STATUS":
                    self._set_conn_status("connected", "Connected")
                    self.connect_time = time.time()
                    self._add_alert("Robot Start Running", "Robot online, menunggu data GPS dari ROS...", "success")
                elif kind == "ERROR":
                    self._set_conn_status("error", "Error")
                    self._add_alert("Serial Error", payload, "danger")
                elif kind == "LINE":
                    try:
                        self._handle_line(payload)
                    except Exception:
                        print(traceback.format_exc())
        except queue.Empty:
            pass
        except Exception:
            print(traceback.format_exc())

    def _handle_line(self, line):
        if not line.startswith("DATA,"):
            return

        parts = line.split(",")
        if len(parts) < FIELDS_EXPECTED:
            return

        values = parts[1:FIELDS_EXPECTED]
        data = dict(zip(FIELD_NAMES, values))

        try:
            seq = int(data["seq"])
            rssi = float(data["rssi"])
            snr = float(data["snr"])
            lat = float(data["lat"])
            lon = float(data["lon"])
            mx, my, mz = float(data["mx"]), float(data["my"]), float(data["mz"])
            heading = float(data["heading"])
            dist_cm = int(data["dist_cm"])
            bumper = int(data["bumper"])
            gx, gy, gz = float(data["gx"]), float(data["gy"]), float(data["gz"])
            ax, ay, az = float(data["ax"]), float(data["ay"]), float(data["az"])
            dt = int(data["dt"])
            pkt_count = int(data["packetCount"])
        except (ValueError, KeyError):
            return

        if not self.got_first_packet:
            self.got_first_packet = True

        self.card_rssi.set_value(f"{rssi:.0f} dBm")
        self.card_snr.set_value(f"{snr:.1f} dB")

        # -- statistik paket dihitung sendiri di GUI --
        # Sebelumnya "ACK" cuma nampilin field packetCount mentah dari
        # serial (gak pernah reset, numpuk terus dari device nyala), dan
        # "Paket hilang" dihitung dari seq - packetCount yang ternyata
        # selalu sinkron (packetCount cuma echo dari seq pengirim), jadi
        # loss selalu 0 walau paketnya kedrop. Sekarang dihitung dari gap
        # nomor seq yang BENERAN kita terima di sini.
        # SEQ ditampilin APA ADANYA (nomor mentah dari transmitter), biar
        # bisa langsung dicocokin sama serial monitor di sisi transmitter --
        # gak di-offset/direset ke 0, karena transmitter-nya juga gak reset.
        self._local_rx_count += 1

        if self._last_seq is not None:
            gap = seq - self._last_seq - 1
            if gap > 0:
                self._loss_total += gap
            # gap < 0 (seq balik ke kecil, misal device restart) dibiarkan
            # aja, gak dihitung sebagai loss negatif -- cuma nunggu nomor
            # jalan maju lagi secara normal buat deteksi gap berikutnya.
        self._last_seq = seq

        self.card_seq.set_value(f"{seq}")
        self.card_pktcount.set_value(f"{self._local_rx_count}")
        self.card_loss.set_value(f"{self._loss_total}")

        self.card_lat.setText(f"Lat: {lat:.6f}")
        self.card_lon.setText(f"Long: {lon:.6f}")

        self.card_accel.set_value(f"{ax:.2f} / {ay:.2f} / {az:.2f}")
        self.card_gyro.set_value(f"{gx:.1f} / {gy:.1f} / {gz:.1f}")

        # TODO speed dari sensor asli: kalau ada wheel encoder / speed sensor,
        # ganti bagian ini biar gak cuma estimasi dari selisih GPS.
        now_t = time.monotonic()
        if self._last_fix is not None and lat != 0.0 and lon != 0.0:
            plat, plon, pt = self._last_fix
            dt_s = now_t - pt
            if dt_s > 0:
                dist_m = haversine_m(plat, plon, lat, lon)
                speed = dist_m / dt_s
                self.card_speed.set_value(f"{speed:.2f} m/s")
        if lat != 0.0 and lon != 0.0:
            self._last_fix = (lat, lon, now_t)

        if dist_cm < 0:
            self.card_dist.set_value("timeout")
        else:
            color = COLOR_BAD if dist_cm < 15 else (COLOR_WARN if dist_cm < OBSTACLE_WARN_CM else COLOR_OK)
            self.card_dist.set_value(f"{dist_cm} cm", color=color)

            zone = "warn" if dist_cm < OBSTACLE_WARN_CM else "ok"
            if zone == "warn" and self._prev_obstacle_zone != "warn":
                self._add_alert("Obstacle Warning", f"Distance: {dist_cm} cm", "warn")
            self._prev_obstacle_zone = zone

        if bumper == 1:
            self.bumper_indicator.set_value("kena woe!!", color=COLOR_BAD)
            if self._prev_bumper == 0:
                self._add_alert("hati hati dong kalo pake robot!", "kena oi", "bahaya")
        else:
            self.bumper_indicator.set_value("Aman", color=COLOR_OK)
        self._prev_bumper = bumper

        # Mode 1 port: guimain sendiri yang publish ke ROS (gantiin readserial.py)
        if self.self_publish_ros and self.node is not None:
            try:
                self.node.publish_from_dashboard(lat, lon, heading, mx, my, mz, gx, gy, gz, ax, ay, az)
            except Exception:
                print(traceback.format_exc())

    # ---------------------------------------------------
    def closeEvent(self, event):
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.stop()
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        super().closeEvent(event)


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()

    app = QApplication(sys.argv)
    node = GPSNode()
    window = MainGUIWindow(node=node)

    node.data_signal.connect(window.update_map)
    node.compass_signal.connect(window.update_compass)
    node.heading_map_signal.connect(window.update_heading_map)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()