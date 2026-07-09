import sys
import os
import threading
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel, QRadioButton, QPushButton, QButtonGroup
)
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygon
from PyQt5.QtCore import QObject, pyqtSignal, QUrl, QPoint, Qt
from PyQt5.QtWebEngineWidgets import QWebEngineView


# =========================================================
# COMPASS WIDGET — no background, overlay style
# =========================================================
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.setMinimumSize(160, 160)

    def set_heading(self, heading):
        self.heading = heading
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 10

        # Lingkaran tipis semi-transparan
        painter.setPen(QPen(QColor("#cccccc"), 2))
        painter.setBrush(QBrush(QColor("#f9f9f9")))
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # Label N/S/E/W
        for label, angle in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            rad = math.radians(angle)
            lx = cx + int((r - 14) * math.sin(rad))
            ly = cy - int((r - 14) * math.cos(rad))
            color = QColor("#e74c3c") if label == "N" else QColor("#444444")
            painter.setPen(QPen(color))
            painter.drawText(lx - 6, ly + 6, label)

        # Tick marks
        painter.setPen(QPen(QColor(180, 180, 180, 200), 1))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x1 = cx + int((r - 3)  * math.sin(rad))
            y1 = cy - int((r - 3)  * math.cos(rad))
            x2 = cx + int((r - 9) * math.sin(rad))
            y2 = cy - int((r - 9) * math.cos(rad))
            painter.drawLine(x1, y1, x2, y2)

        # Jarum
        painter.translate(cx, cy)
        painter.rotate(self.heading)

        needle_len = r - 18
        tail_len   = r - 45

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#e74c3c")))
        painter.drawPolygon(QPolygon([
            QPoint(0, -needle_len), QPoint(6, 5),
            QPoint(0, 0),           QPoint(-6, 5)
        ]))

        painter.setBrush(QBrush(QColor("#aaaaaa")))
        painter.drawPolygon(QPolygon([
            QPoint(0, tail_len), QPoint(5, -3),
            QPoint(0, 0),        QPoint(-5, -3)
        ]))

        painter.setBrush(QBrush(QColor("#333333")))
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.drawEllipse(-5, -5, 10, 10)
        painter.resetTransform()

        # Teks heading
        painter.setPen(QPen(QColor("#222222")))
        painter.drawText(cx - 35, cy + r + 16, f"{self.heading:.1f}°")


# =========================================================
# GPS WINDOW
# =========================================================
class GPSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SECURITY PATROL ROBOT RE-054 TRACKING")
        self.resize(1100, 700)
        self._current_mode = 'raw'

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Panel tombol
        panel_widget = QWidget()
        panel_widget.setFixedHeight(40)
        panel = QHBoxLayout(panel_widget)
        panel.setContentsMargins(8, 4, 8, 4)
        panel.addWidget(QLabel("Mode Tampilan:"))

        self.rb_raw  = QRadioButton("Raw GPS (MERAH)")
        self.rb_raw.setChecked(True)
        self.rb_raw.setStyleSheet("color:red; font-weight:bold")

        self.rb_ekf  = QRadioButton("GPS(maf+threshold), Compass+IMU(CF)")
        self.rb_ekf.setStyleSheet("color:green; font-weight:bold")

        self.rb_none = QRadioButton("NO TRACKING")
        self.rb_none.setStyleSheet("color:blue; font-weight:bold")

        group = QButtonGroup()
        group.addButton(self.rb_raw)
        group.addButton(self.rb_ekf)
        group.addButton(self.rb_none)

        self.btn_reset = QPushButton("Reset Tracking")
        self.btn_reset.setStyleSheet("background:#ffcccc; font-weight:bold")

        panel.addWidget(self.rb_raw)
        panel.addWidget(self.rb_ekf)
        panel.addWidget(self.rb_none)
        panel.addWidget(self.btn_reset)
        panel.addStretch()

        main_layout.addWidget(panel_widget)

        # ── Container peta + compass overlay
        map_container = QWidget()
        map_container.setContentsMargins(0, 0, 0, 0)
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(0, 0, 0, 0)

        self.web = QWebEngineView(map_container)
        map_layout.addWidget(self.web)

        # Compass overlay — ditaruh di atas peta, pojok kanan bawah
        self.compass = CompassWidget(map_container)
        self.compass.setFixedSize(160, 180)
        self.compass.raise_()

        main_layout.addWidget(map_container)

        # Posisi compass akan diset saat resize
        self._map_container = map_container

        self.map_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "tracking_map.html"
        )
        self.init_map_html(1.118489, 104.048447)
        self.web.setUrl(QUrl.fromLocalFile(self.map_file))

        self.rb_raw.toggled.connect(lambda checked:  self._on_switch("raw")  if checked else None)
        self.rb_ekf.toggled.connect(lambda checked:  self._on_switch("ekf")  if checked else None)
        self.rb_none.toggled.connect(lambda checked: self._on_switch("none") if checked else None)
        self.btn_reset.clicked.connect(self.reset_path)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_compass()

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_compass()

    def _reposition_compass(self):
        if hasattr(self, 'compass') and hasattr(self, '_map_container'):
            cw = self.compass.width()
            ch = self.compass.height()
            mw = self._map_container.width()
            mh = self._map_container.height()
            # Pojok kiri bawah
            self.compass.move(mw - cw - 12, mh - ch - 12)
            self.compass.raise_()

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
        self.web.page().runJavaScript(
            f"updatePosition('{mode}', {lat}, {lon}, {heading});"
        )

    def update_compass(self, mode, heading):
        # Hanya update compass kalau mode-nya cocok
        if mode == self._current_mode or self._current_mode == 'none':
            self.compass.set_heading(heading)

    def update_heading_map(self, mode, heading):
        self.web.page().runJavaScript(
            f"updateHeading('{mode}', {heading});"
        )

    def switch(self, mode):
        self.web.page().runJavaScript(f"switchLayer('{mode}');")

    def reset_path(self):
        self.web.page().runJavaScript("clearPath();")


# =========================================================
# ROS NODE
# =========================================================
class GPSNode(Node, QObject):
    data_signal        = pyqtSignal(str, float, float, float)
    # mode, heading
    compass_signal     = pyqtSignal(str, float)
    heading_map_signal = pyqtSignal(str, float)

    def __init__(self):
        Node.__init__(self, 'gps_gui_node')
        QObject.__init__(self)

        self._heading_raw = 0.0
        self._heading_cf  = 0.0

        self.create_subscription(NavSatFix, '/gps/fix',
                                 self.cb_raw, 10)
        self.create_subscription(NavSatFix, '/gps/filtered',
                                 self.cb_ekf, 10)
        self.create_subscription(Float32, '/compass/heading',
                                 self.cb_heading_raw, 10)
        self.create_subscription(Float32, '/compass/heading_cf',
                                 self.cb_heading_cf, 10)

        self.get_logger().info("GPS GUI node ready")

    def cb_heading_raw(self, msg):
        self._heading_raw = float(msg.data)
        self.heading_map_signal.emit('raw', self._heading_raw)
        # Kirim ke compass dengan tag 'raw'
        self.compass_signal.emit('raw', self._heading_raw)

    def cb_heading_cf(self, msg):
        self._heading_cf = float(msg.data)
        self.heading_map_signal.emit('ekf', self._heading_cf)
        # Kirim ke compass dengan tag 'ekf'
        self.compass_signal.emit('ekf', self._heading_cf)

    def cb_raw(self, msg):
        if msg.latitude != 0.0:
            self.data_signal.emit(
                'raw', msg.latitude, msg.longitude, self._heading_raw
            )

    def cb_ekf(self, msg):
        if msg.latitude != 0.0:
            self.data_signal.emit(
                'ekf', msg.latitude, msg.longitude, self._heading_cf
            )


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()

    app = QApplication(sys.argv)
    window = GPSWindow()

    node = GPSNode()
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