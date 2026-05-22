import sys
import os

# ── Must be set BEFORE Qt / WebEngine initialises ──────────────
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
    "--disable-gpu "
    "--disable-software-rasterizer "
    "--disable-gpu-compositing"
)
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

import time
import random
import math
import threading
import numpy as np
from datetime import datetime

# ── PyQt5 imports (ganti dari PyQt6) ──────────────────────────
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QStackedWidget,
                             QFrame, QScrollArea, QGridLayout, QSizePolicy,
                             QTextEdit, QSpacerItem)
from PyQt5.QtGui import (QPixmap, QImage, QPainter, QPen, QBrush, QColor,
                          QFont, QPolygonF, QFontMetrics)
from PyQt5.QtCore import (Qt, QUrl, QTimer, QPropertyAnimation, QRect,
                           pyqtSignal, QObject, QPointF, QRectF)

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


# ──────────────────────────────────────────────────────────────
POLIBATAM_ROUTE = [
    (1.11800, 104.04820), (1.11820, 104.04860), (1.11850, 104.04900),
    (1.11880, 104.04930), (1.11900, 104.04960), (1.11920, 104.04990),
    (1.11940, 104.05010), (1.11960, 104.05030), (1.11970, 104.05000),
    (1.11960, 104.04970), (1.11940, 104.04940), (1.11910, 104.04910),
    (1.11880, 104.04880), (1.11850, 104.04860), (1.11820, 104.04840),
    (1.11800, 104.04820),
]

WP_LABELS  = ["WP 1", "WP 2", "WP 3", "WP 4"]
WP_INDICES = [1, 3, 7, 13]

CAMERA_INDEX = 0
PRIMARY   = "#120078"
BG_MAIN   = "#D9D9D9"
WHITE     = "#FFFFFF"
GREEN_OK  = "#00C853"
RED_ALERT = "#E53935"
ORANGE    = "#F57C00"


# ══════════════════════════════════════════════════════════════
# DUMMY ARDUINO
# ══════════════════════════════════════════════════════════════
class DummyArduino:
    def __init__(self):
        self._ppr          = 600
        self._pulse_step   = 5
        self._enc_pulse    = 0
        self._enc_dir      = 1
        self._enc_rpm      = 0.0
        self._enc_start    = time.time()
        self._ultra        = 100
        self._imu          = 0.0
        self._route_idx    = 0
        self._battery      = 78.0
        self._uptime_start = time.time()
        self._last_arrived_wp = -1

    def _tick_encoder(self):
        now        = time.time()
        elapsed_ms = int((now - self._enc_start) * 1000)
        self._enc_dir = -1 if (elapsed_ms // 5000) % 2 == 1 else 1
        for _ in range(20):
            self._enc_pulse += self._pulse_step * self._enc_dir
        self._enc_rpm = (self._pulse_step * 20.0 * 60.0) / self._ppr

    def next_gps(self):
        coord = POLIBATAM_ROUTE[self._route_idx % len(POLIBATAM_ROUTE)]
        lat = coord[0] + random.uniform(-0.00005, 0.00005)
        lng = coord[1] + random.uniform(-0.00005, 0.00005)
        self._route_idx += 1
        return lat, lng

    def generate(self):
        self._tick_encoder()
        self._ultra   = max(10, min(150, self._ultra + random.randint(-8, 8)))
        self._imu     = (self._imu + random.uniform(-5, 10)) % 360
        self._battery = max(0, self._battery - random.uniform(0, 0.02))
        wheel_radius  = 0.05
        speed_ms      = round((self._enc_rpm / 60.0) * 2 * math.pi * wheel_radius, 2)
        uptime_s      = int(time.time() - self._uptime_start)
        return {
            "ultra":     self._ultra,
            "enc_pulse": self._enc_pulse,
            "enc_rpm":   round(self._enc_rpm, 2),
            "enc_speed": speed_ms,
            "imu":       int(self._imu),
            "battery":   round(self._battery, 1),
            "uptime":    uptime_s,
        }

    def patrol_status(self):
        route_idx = self._route_idx % len(POLIBATAM_ROUTE)
        for i, wi in enumerate(WP_INDICES):
            if route_idx == wi:
                if self._last_arrived_wp != i:
                    self._last_arrived_wp = i
                    return ("arrived", WP_LABELS[i])
                break
        next_i = None
        for i, wi in enumerate(WP_INDICES):
            if wi > route_idx:
                next_i = i
                break
        if next_i is None:
            next_i = 0
        return ("heading", WP_LABELS[next_i])


# ══════════════════════════════════════════════════════════════
# CAMERA DETECTOR
# ══════════════════════════════════════════════════════════════
class DetectorSignals(QObject):
    result = pyqtSignal(object, int)

class PersonDetector:
    def __init__(self):
        self.signals  = DetectorSignals()
        self._running = False
        self._cap     = None
        self._thread  = None

    def start(self):
        if not HAS_CV2:
            return
        cap = None
        candidates = list(dict.fromkeys([CAMERA_INDEX, 0, 1, 2, 3]))
        for idx in candidates:
            try:
                # Ubuntu: hapus CAP_DSHOW (Windows only), pakai default
                c = cv2.VideoCapture(idx)
            except Exception:
                c = cv2.VideoCapture(idx)
            if c and c.isOpened():
                cap = c
                break
            if c:
                c.release()
        if cap is None:
            return
        self._cap     = cap
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._cap:
            self._cap.release()

    def _loop(self):
        consecutive_failures = 0
        while self._running:
            if not self._cap or not self._cap.isOpened():
                time.sleep(0.1)
                continue
            ret, frame = self._cap.read()
            if not ret:
                consecutive_failures += 1
                time.sleep(min(0.1 * consecutive_failures, 1.0))
                if consecutive_failures > 30:
                    break
                continue
            consecutive_failures = 0
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.signals.result.emit(rgb, 0)
            time.sleep(0.033)


# ══════════════════════════════════════════════════════════════
# BATTERY BAR
# ══════════════════════════════════════════════════════════════
def make_battery_pixmap(pct, w=80, h=14):
    px = QPixmap(w, h)
    px.fill(Qt.transparent)  # PyQt5: Qt.transparent bukan Qt.GlobalColor.transparent
    p = QPainter(px)
    p.setRenderHint(QPainter.Antialiasing)  # PyQt5: tanpa RenderHint.
    p.setBrush(QBrush(QColor("#E0E0E0")))
    p.setPen(Qt.NoPen)  # PyQt5: Qt.NoPen bukan Qt.PenStyle.NoPen
    p.drawRoundedRect(0, 0, w, h, 4, 4)
    fill_w = max(4, int(w * pct / 100))
    col = (QColor(GREEN_OK) if pct > 50 else
           (QColor("#FFD600") if pct > 20 else QColor(RED_ALERT)))
    p.setBrush(QBrush(col))
    p.drawRoundedRect(0, 0, fill_w, h, 4, 4)
    p.end()
    return px


# ══════════════════════════════════════════════════════════════
# MAIN WINDOW
# ══════════════════════════════════════════════════════════════
class SecurityRobotPyQt(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Security Patrol Robot Dashboard - RE-054")
        self.setMinimumSize(1100, 720)

        self.arduino       = DummyArduino()
        self.detector      = PersonDetector()

        self.gps_log       = []
        self._alert_rows   = []

        self._current_lat  = POLIBATAM_ROUTE[0][0]
        self._current_lng  = POLIBATAM_ROUTE[0][1]
        self._person_count = 0
        self._map_loaded   = False
        self._map_loading  = False
        self._robot_state  = "PATROL"
        self._patrol_label = "Heading to WP 1"

        self._apply_stylesheet()
        self._build_skeleton()
        self._setup_header()
        self._setup_menu()
        self._create_pages()
        self.show_page("Home")
        self._start_timers()

        if HAS_CV2:
            self.detector.signals.result.connect(self._on_camera_frame)
            self.detector.start()
        else:
            QTimer.singleShot(300, self._show_no_cam)

    def _apply_stylesheet(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {BG_MAIN}; border: none; }}
            QScrollBar:vertical {{ border:none; background:#E0E0E0;
                                   width:8px; border-radius:4px; }}
            QScrollBar::handle:vertical {{ background:{PRIMARY};
                                           min-height:28px; border-radius:4px; }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {{ height:0px; }}
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical {{ background:none; }}
        """)

    def _build_skeleton(self):
        self.central = QWidget()
        self.setCentralWidget(self.central)
        self.main_vbox = QVBoxLayout(self.central)
        self.main_vbox.setContentsMargins(0, 0, 0, 0)
        self.main_vbox.setSpacing(0)

        self.clock_label = QLabel()
        self.clock_label.setStyleSheet("color:white; font-size:15px;")

        self.stack = QStackedWidget()
        self.stack.setStyleSheet("background:transparent; border:none;")

    def _start_timers(self):
        self._sensor_timer = QTimer(self)
        self._sensor_timer.timeout.connect(self._update_sensors)
        self._sensor_timer.start(1000)

        self._gps_timer = QTimer(self)
        self._gps_timer.timeout.connect(self._update_gps)
        self._gps_timer.start(5000)

        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start(1000)

        self._tick_clock()
        self._update_gps()

    # ══════════════════════════════════════════════════════════
    # HEADER
    # ══════════════════════════════════════════════════════════
    def _setup_header(self):
        hdr = QFrame()
        hdr.setFixedHeight(60)
        hdr.setStyleSheet(f"background:{PRIMARY}; border:none;")
        lay = QHBoxLayout(hdr)
        lay.setContentsMargins(16, 0, 24, 0)
        lay.setSpacing(12)

        menu_btn = QPushButton("☰")
        menu_btn.setFixedSize(44, 44)
        menu_btn.setCursor(Qt.PointingHandCursor)  # PyQt5: Qt.PointingHandCursor
        menu_btn.setStyleSheet(
            "color:white; font-size:22px; border:none; background:transparent;")
        menu_btn.clicked.connect(self.toggle_menu)

        title = QLabel("Security Patrol Robot")
        title.setStyleSheet(
            "color:white; font-size:22px; font-weight:bold; border:none;")

        self._is_connected = True

        self.conn_btn = QPushButton("CONNECTED")
        self.conn_btn.setFixedHeight(30)
        self.conn_btn.setCursor(Qt.PointingHandCursor)
        self._style_conn_btn()
        self.conn_btn.clicked.connect(self._toggle_connection)

        lay.addWidget(menu_btn)
        lay.addWidget(title)
        lay.addStretch()
        lay.addWidget(self.conn_btn)
        lay.addSpacing(20)
        lay.addWidget(self.clock_label)

        self.main_vbox.addWidget(hdr)
        self.main_vbox.addWidget(self.stack)

        footer = QLabel("2026 Security Patrol Robot System  |  All Right Reserved")
        footer.setAlignment(Qt.AlignCenter)  # PyQt5: Qt.AlignCenter
        footer.setFixedHeight(22)
        footer.setStyleSheet(
            f"background:{BG_MAIN}; color:{PRIMARY}; font-size:10px; border:none;")
        self.main_vbox.addWidget(footer)

    def _tick_clock(self):
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))

    def _style_conn_btn(self):
        if self._is_connected:
            self.conn_btn.setText("CONNECTED")
            self.conn_btn.setStyleSheet(f"""
                QPushButton {{
                    background:{GREEN_OK}; color:white; font-weight:bold;
                    font-size:12px; border-radius:15px; padding:0px 18px;
                    border:none;
                }}
                QPushButton:hover {{ background:#00A846; }}
            """)
        else:
            self.conn_btn.setText("DISCONNECTED")
            self.conn_btn.setStyleSheet(f"""
                QPushButton {{
                    background:{RED_ALERT}; color:white; font-weight:bold;
                    font-size:12px; border-radius:15px; padding:0px 18px;
                    border:none;
                }}
                QPushButton:hover {{ background:#C62828; }}
            """)

    def _toggle_connection(self):
        self._is_connected = not self._is_connected
        self._style_conn_btn()
        if self._is_connected:
            self._sensor_timer.start(1000)
            self._gps_timer.start(5000)
            if HAS_CV2 and not self.detector._running:
                self.detector.start()
            self._add_alert("success", "Reconnected", "Data stream resumed")
        else:
            self._sensor_timer.stop()
            self._gps_timer.stop()
            if HAS_CV2:
                self.detector.stop()
            self.video_label.setText("📡  Disconnected")
            self._add_alert("danger", "Disconnected", "Data stream paused")

    # ══════════════════════════════════════════════════════════
    # DROPDOWN MENU
    # ══════════════════════════════════════════════════════════
    def _setup_menu(self):
        self.menu_drop = QFrame(self)
        self.menu_drop.setGeometry(0, 60, self.width(), 0)
        self.menu_drop.setStyleSheet(
            f"background:white; border-bottom:2px solid {PRIMARY};")
        l = QVBoxLayout(self.menu_drop)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(0)
        for name in ["Home", "Data log"]:
            btn = QPushButton(name)
            btn.setFixedHeight(48)
            btn.setStyleSheet(f"""
                QPushButton {{
                    text-align:left; padding-left:32px;
                    font-weight:bold; font-size:13px;
                    border:none; background:transparent; color:#222;
                }}
                QPushButton:hover {{ background:#F0EEFF; color:{PRIMARY}; }}
            """)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda _, n=name: self.switch_page(n))
            l.addWidget(btn)
        self._menu_open = False

    def toggle_menu(self):
        target_h = 96 if not self._menu_open else 0
        self._anim = QPropertyAnimation(self.menu_drop, b"geometry")
        self._anim.setDuration(200)
        self._anim.setEndValue(QRect(0, 60, self.width(), target_h))
        self._anim.start()
        self._menu_open = not self._menu_open
        self.menu_drop.raise_()

    def switch_page(self, name):
        self.show_page(name)
        if self._menu_open:
            self.toggle_menu()

    def show_page(self, name):
        self.stack.setCurrentWidget(self.pages[name])

    # ══════════════════════════════════════════════════════════
    # PAGES
    # ══════════════════════════════════════════════════════════
    def _create_pages(self):
        self.pages = {}
        self._build_home_page()
        self._build_datalog_page()

    # ══════════════════════════════════════════════════════════
    # HOME PAGE
    # ══════════════════════════════════════════════════════════
    def _build_home_page(self):
        page = QWidget()
        page.setStyleSheet(f"background:{BG_MAIN};")

        outer = QVBoxLayout(page)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addStretch(1)

        center_row = QHBoxLayout()
        center_row.addStretch(1)

        inner = QWidget()
        inner.setStyleSheet("background:transparent;")
        inner_lay = QHBoxLayout(inner)
        inner_lay.setContentsMargins(0, 0, 0, 0)
        inner_lay.setSpacing(48)

        # ── Poster ── pakai path relatif (fix dari hardcoded Windows path)
        poster_lbl = QLabel()
        asset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "pngpatrol.png")
        pix = QPixmap(asset_path)
        if not pix.isNull():
            poster_lbl.setPixmap(
                pix.scaled(370, 500,
                           Qt.KeepAspectRatio,           # PyQt5: Qt.KeepAspectRatio
                           Qt.SmoothTransformation))     # PyQt5: Qt.SmoothTransformation
        else:
            poster_lbl.setText("🤖")
            poster_lbl.setStyleSheet("font-size:120px; background:transparent;")
            poster_lbl.setAlignment(Qt.AlignCenter)
            poster_lbl.setFixedWidth(300)

        # ── Info panel ──
        info = QVBoxLayout()
        info.setSpacing(0)
        info.setContentsMargins(0, 0, 0, 0)

        sub = QLabel("PBL RE-054")
        sub.setStyleSheet(
            "font-size:16px; font-weight:bold; color:#555; background:transparent;")

        title_lbl = QLabel("Security Patrol Robot")
        title_lbl.setStyleSheet(
            "font-size:34px; font-weight:900; color:#000; background:transparent;")

        desc = QLabel(
            "The outdoor security patrol robot is designed to support security tasks "
            "in various open environments. Built with durable mechanical structure and "
            "rated IP65+. It operates reliably in outdoor conditions, while at night. "
            "Equipped with sensors and smart technology, it patrols autonomously, "
            "monitors surroundings, and enhances overall security efficiency.")
        desc.setWordWrap(True)
        desc.setFixedWidth(430)
        desc.setStyleSheet(
            "font-size:14px; color:#333; background:transparent; margin-top:10px;")

        fitur_lbl = QLabel("Fitur")
        fitur_lbl.setStyleSheet(
            "font-size:22px; font-weight:bold; background:transparent; margin-top:22px;")

        icons_row = QHBoxLayout()
        icons_row.setSpacing(32)
        icons_row.setContentsMargins(0, 8, 0, 0)
        icons_row.addWidget(self._feat_icon("📍", "Location"))
        icons_row.addWidget(self._feat_icon("🎥", "Live Cam"))
        icons_row.addWidget(self._feat_icon("📡", "Autonomous"))
        icons_row.addStretch()

        info.addWidget(sub)
        info.addWidget(title_lbl)
        info.addWidget(desc)
        info.addWidget(fitur_lbl)
        info.addLayout(icons_row)
        info.addStretch()

        inner_lay.addWidget(poster_lbl, 0, Qt.AlignVCenter)  # PyQt5: Qt.AlignVCenter
        inner_lay.addLayout(info)

        center_row.addWidget(inner)
        center_row.addStretch(1)
        outer.addLayout(center_row)
        outer.addStretch(1)

        self.pages["Home"] = page
        self.stack.addWidget(page)

    def _feat_icon(self, emoji, label):
        w = QWidget()
        w.setStyleSheet("background:transparent;")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        e = QLabel(emoji)
        e.setStyleSheet("font-size:30px; background:transparent;")
        l = QLabel(label)
        l.setStyleSheet("font-size:12px; color:#555; background:transparent;")
        lay.addWidget(e, 0, Qt.AlignCenter)
        lay.addWidget(l, 0, Qt.AlignCenter)
        return w

    # ══════════════════════════════════════════════════════════
    # DATA LOG PAGE
    # ══════════════════════════════════════════════════════════
    def _build_datalog_page(self):
        page = QWidget()
        page.setStyleSheet(f"background:{BG_MAIN};")
        outer = QVBoxLayout(page)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(10)

        row_top = QHBoxLayout()
        row_top.setSpacing(10)
        row_top.addWidget(self._build_map_card(),    45)
        row_top.addWidget(self._build_camera_card(), 55)
        outer.addLayout(row_top, 60)

        row_bot = QHBoxLayout()
        row_bot.setSpacing(10)
        row_bot.addWidget(self._build_manual_card(),  25)
        row_bot.addWidget(self._build_status_card(),  33)
        row_bot.addWidget(self._build_alerts_card(),  42)
        outer.addLayout(row_bot, 40)

        self.pages["Data log"] = page
        self.stack.addWidget(page)

    # ── Card shell helper ─────────────────────────────────────
    def _card_shell(self, title_text):
        card = QFrame()
        card.setStyleSheet(
            f"background:{WHITE}; border-radius:10px; border:none;")
        card_lay = QVBoxLayout(card)
        card_lay.setContentsMargins(0, 0, 0, 0)
        card_lay.setSpacing(0)

        hdr = QFrame()
        hdr.setFixedHeight(36)
        hdr.setStyleSheet(f"""
            background:{PRIMARY};
            border-top-left-radius:10px;
            border-top-right-radius:10px;
        """)
        hlay = QHBoxLayout(hdr)
        hlay.setContentsMargins(14, 0, 14, 0)
        hlay.setSpacing(8)

        ht = QLabel(title_text)
        ht.setStyleSheet(
            "color:white; font-weight:bold; font-size:13px; background:transparent;")
        hlay.addWidget(ht)
        hlay.addStretch()

        body = QWidget()
        body.setStyleSheet(f"background:{WHITE};")

        card_lay.addWidget(hdr)
        card_lay.addWidget(body)
        return card, hlay, body

    # ══════════════════════════════════════════════════════════
    # MAP CARD
    # ══════════════════════════════════════════════════════════
    def _build_map_card(self):
        card, hlay, body = self._card_shell("📍  Location")
        blay = QVBoxLayout(body)
        blay.setContentsMargins(8, 8, 8, 8)

        if HAS_WEBENGINE:
            self.map_view = QWebEngineView()
        else:
            self.map_view = QLabel("🗺️  Map tidak tersedia\nKoordinat: loading...")
            self.map_view.setAlignment(Qt.AlignCenter)
            self.map_view.setStyleSheet(
                "background:#e8f0fe; border-radius:6px; font-size:13px; color:#333;")

        self.map_view.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)  # PyQt5: tanpa Policy.
        blay.addWidget(self.map_view)
        return card

    def _on_map_load_finished(self, ok):
        self._map_loading = False
        if ok:
            self._map_loaded = True
            if hasattr(self, '_pending_gps'):
                lat, lng = self._pending_gps
                del self._pending_gps
                self.map_view.page().runJavaScript(f"moveMarker({lat},{lng});")

    def _load_map_once(self, lat, lng):
        route_js = "[" + ",".join(
            f"[{r[0]},{r[1]}]" for r in POLIBATAM_ROUTE) + "]"
        wps_js = "[" + ",".join(
            f"[{POLIBATAM_ROUTE[i][0]},{POLIBATAM_ROUTE[i][1]},'{WP_LABELS[n]}']"
            for n, i in enumerate(WP_INDICES)) + "]"

        html = f"""<!DOCTYPE html><html><head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
html,body,#map{{margin:0;padding:0;height:100%;width:100%;}}
#compass{{
  position:absolute;bottom:36px;right:10px;z-index:1000;
  width:52px;height:52px;pointer-events:none;
}}
#compass svg{{width:52px;height:52px;}}
</style>
</head><body><div id="map"></div>
<div id="compass">
<svg viewBox="0 0 52 52" xmlns="http://www.w3.org/2000/svg">
  <circle cx="26" cy="26" r="25" fill="white" stroke="#ccc" stroke-width="1.5" opacity="0.92"/>
  <polygon points="26,6 30,26 26,22 22,26" fill="#E53935"/>
  <polygon points="26,46 30,26 26,30 22,26" fill="#555"/>
  <circle cx="26" cy="26" r="3" fill="#333"/>
  <text x="26" y="15" text-anchor="middle" font-size="7" font-weight="bold" fill="#E53935" font-family="Arial">N</text>
  <text x="26" y="42" text-anchor="middle" font-size="7" fill="#555" font-family="Arial">S</text>
  <text x="10" y="29" text-anchor="middle" font-size="7" fill="#555" font-family="Arial">W</text>
  <text x="42" y="29" text-anchor="middle" font-size="7" fill="#555" font-family="Arial">E</text>
</svg>
</div>
<script>
var map=L.map('map',{{zoomControl:true}}).setView([{lat},{lng}],18);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{
  maxZoom:19,attribution:'© OpenStreetMap'}}).addTo(map);

var route={route_js};
L.polyline(route,{{color:'{PRIMARY}',weight:3,
  dashArray:'8,5',opacity:0.9}}).addTo(map);

var wps={wps_js};
wps.forEach(function(wp){{
  var ic=L.divIcon({{
    html:'<div style="background:#7B1FA2;color:white;font-size:10px;font-weight:bold;'
        +'border-radius:50%;width:24px;height:24px;display:flex;align-items:center;'
        +'justify-content:center;border:2px solid white;'
        +'box-shadow:0 1px 4px rgba(0,0,0,.5)">'+wp[2].replace('WP ','')+'</div>',
    iconSize:[24,24],iconAnchor:[12,12],className:''}});
  L.marker([wp[0],wp[1]],{{icon:ic}}).bindTooltip(wp[2]).addTo(map);
}});

var robotIcon=L.divIcon({{
  html:'<div style="background:{PRIMARY};width:22px;height:22px;border-radius:50%;'
      +'border:3px solid white;box-shadow:0 0 8px rgba(0,0,0,.5)"></div>',
  iconSize:[22,22],iconAnchor:[11,11],className:''}});
var marker=L.marker([{lat},{lng}],{{icon:robotIcon}}).addTo(map);
marker.bindTooltip('Robot RE-054').openTooltip();

function moveMarker(lat,lng){{
  var ll=L.latLng(lat,lng);
  marker.setLatLng(ll);
  map.panTo(ll);
}}
</script></body></html>"""
        self.map_view.loadFinished.connect(self._on_map_load_finished)
        self._map_loading = True
        self.map_view.setHtml(html, QUrl("https://unpkg.com/"))

    def _update_gps(self):
        lat, lng = self.arduino.next_gps()
        self._current_lat = lat
        self._current_lng = lng

        event, wp_label = self.arduino.patrol_status()

        if event == "arrived":
            self._patrol_label = f"Arrived at {wp_label}"
            self._add_alert("success", f"Arrived at {wp_label}", "Checkpoint reached — continuing patrol")
        else:
            self._patrol_label = f"Heading to {wp_label}"
            self._add_alert("info", f"Heading to {wp_label}", "Robot on patrol route")

        if hasattr(self, '_st_mission'):
            is_arrived = event == "arrived"
            col = GREEN_OK if is_arrived else "#1565C0"
            self._st_mission.setText(self._patrol_label)
            self._st_mission.setStyleSheet(
                f"font-size:13px; font-weight:bold; color:{col}; background:transparent;")

        if HAS_WEBENGINE:
            if not self._map_loaded and not self._map_loading:
                self._load_map_once(lat, lng)
            elif not self._map_loaded and self._map_loading:
                self._pending_gps = (lat, lng)
            else:
                self.map_view.page().runJavaScript(f"moveMarker({lat},{lng});")
        else:
            self.map_view.setText(f"🗺️  {self._patrol_label}")

    # ══════════════════════════════════════════════════════════
    # LIVE CAMERA CARD
    # ══════════════════════════════════════════════════════════
    def _build_camera_card(self):
        card, hlay, body = self._card_shell("🎥  Live camera")
        blay = QVBoxLayout(body)
        blay.setContentsMargins(8, 8, 8, 8)

        self.video_label = QLabel("📷  Memuat kamera...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet(
            "background:#111; border-radius:6px; color:#aaa; font-size:14px;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        blay.addWidget(self.video_label)

        fs_btn = QPushButton("⛶")
        fs_btn.setFixedSize(26, 26)
        fs_btn.setCursor(Qt.PointingHandCursor)
        fs_btn.setStyleSheet(
            "color:white; font-size:13px; border:none; background:transparent;")
        fs_btn.clicked.connect(self._open_camera_fullscreen)
        hlay.addWidget(fs_btn)

        return card

    def _on_camera_frame(self, rgb_frame, _person_count):
        h, w, ch = rgb_frame.shape
        img = QImage(rgb_frame.data, w, h, ch * w, QImage.Format_RGB888)  # PyQt5: Format_RGB888
        img = img.mirrored(horizontal=True, vertical=False)
        pix = QPixmap.fromImage(img)
        lbl_size = self.video_label.size()
        self.video_label.setPixmap(
            pix.scaled(
                lbl_size,
                Qt.KeepAspectRatioByExpanding,  # PyQt5: Qt.KeepAspectRatioByExpanding
                Qt.SmoothTransformation))
        if hasattr(self, '_video_label_full') and self._video_label_full.isVisible():
            self._video_label_full.setPixmap(
                pix.scaled(
                    self._video_label_full.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation))

    def _show_no_cam(self):
        self.video_label.setText("❌  Kamera tidak tersedia\n(install opencv-python)")

    def _open_camera_fullscreen(self):
        if not hasattr(self, '_cam_overlay'):
            overlay = QFrame(self.central)
            overlay.setStyleSheet("background:#000; border:none;")
            overlay.setGeometry(0, 0, self.central.width(), self.central.height())

            lay = QVBoxLayout(overlay)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(0)

            top_bar = QFrame()
            top_bar.setFixedHeight(44)
            top_bar.setStyleSheet(f"background:{PRIMARY}; border:none;")
            tb_lay = QHBoxLayout(top_bar)
            tb_lay.setContentsMargins(16, 0, 16, 0)

            title = QLabel("🎥  Live Camera — Fullscreen")
            title.setStyleSheet(
                "color:white; font-weight:bold; font-size:13px; background:transparent;")
            tb_lay.addWidget(title)
            tb_lay.addStretch()

            min_btn = QPushButton("✕  Minimize")
            min_btn.setFixedHeight(30)
            min_btn.setCursor(Qt.PointingHandCursor)
            min_btn.setStyleSheet(f"""
                QPushButton {{ background:#3B30B8; color:white; border-radius:6px;
                               font-size:11px; font-weight:bold; border:none; padding:0 14px; }}
                QPushButton:hover {{ background:#5548D0; }}
            """)
            min_btn.clicked.connect(self._close_camera_fullscreen)
            tb_lay.addWidget(min_btn)

            lay.addWidget(top_bar)

            self._video_label_full = QLabel()
            self._video_label_full.setAlignment(Qt.AlignCenter)
            self._video_label_full.setStyleSheet("background:#000;")
            self._video_label_full.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lay.addWidget(self._video_label_full)

            self._cam_overlay = overlay

        self._cam_overlay.setGeometry(0, 0, self.central.width(), self.central.height())
        self._cam_overlay.show()
        self._cam_overlay.raise_()

    def _close_camera_fullscreen(self):
        if hasattr(self, '_cam_overlay'):
            self._cam_overlay.hide()

    # ══════════════════════════════════════════════════════════
    # MANUAL CONTROL CARD
    # ══════════════════════════════════════════════════════════
    def _build_manual_card(self):
        card, hlay, body = self._card_shell("🕹️  Manual control")
        blay = QVBoxLayout(body)
        blay.setContentsMargins(16, 16, 16, 16)
        blay.addStretch()

        grid = QGridLayout()
        grid.setSpacing(8)

        arrow_ss = f"""
            QPushButton {{
                background:#E8E8E8; border-radius:8px;
                font-size:20px; color:#555; border:none;
            }}
            QPushButton:pressed {{ background:#BDBDBD; }}
        """
        stop_ss = f"""
            QPushButton {{
                background:{RED_ALERT}; border-radius:8px;
                font-size:12px; font-weight:bold; color:white; border:none;
            }}
            QPushButton:pressed {{ background:#B71C1C; }}
        """

        self._btn_up    = QPushButton("↑");    self._btn_up.setFixedSize(48, 48)
        self._btn_left  = QPushButton("←");    self._btn_left.setFixedSize(48, 48)
        self._btn_stop  = QPushButton("STOP"); self._btn_stop.setFixedSize(48, 48)
        self._btn_right = QPushButton("→");    self._btn_right.setFixedSize(48, 48)
        self._btn_down  = QPushButton("↓");    self._btn_down.setFixedSize(48, 48)

        for b in [self._btn_up, self._btn_left, self._btn_right, self._btn_down]:
            b.setStyleSheet(arrow_ss)
        self._btn_stop.setStyleSheet(stop_ss)

        self.dpad_btns = {
            "up":    self._btn_up,
            "left":  self._btn_left,
            "right": self._btn_right,
            "down":  self._btn_down,
        }

        self._btn_up.clicked.connect(    lambda: self._manual_move("up"))
        self._btn_left.clicked.connect(  lambda: self._manual_move("left"))
        self._btn_right.clicked.connect( lambda: self._manual_move("right"))
        self._btn_down.clicked.connect(  lambda: self._manual_move("down"))
        self._btn_stop.clicked.connect(  lambda: self._manual_move(None))

        grid.addWidget(self._btn_up,    0, 1)
        grid.addWidget(self._btn_left,  1, 0)
        grid.addWidget(self._btn_stop,  1, 1)
        grid.addWidget(self._btn_right, 1, 2)
        grid.addWidget(self._btn_down,  2, 1)

        wrap = QWidget()
        wrap.setStyleSheet("background:transparent;")
        wrap.setLayout(grid)
        blay.addWidget(wrap, 0, Qt.AlignCenter)
        blay.addStretch()
        return card

    _ARROW_SS = f"""
        QPushButton {{ background:#E8E8E8; border-radius:8px;
                       font-size:20px; color:#555; border:none; }}
    """
    _ACTIVE_SS = f"""
        QPushButton {{ background:{PRIMARY}; border-radius:8px;
                       font-size:20px; color:white; border:none; }}
    """

    def _manual_move(self, direction):
        for name, btn in self.dpad_btns.items():
            btn.setStyleSheet(
                self._ACTIVE_SS if name == direction
                else f"QPushButton {{ background:#E8E8E8; border-radius:8px; "
                     f"font-size:20px; color:#555; border:none; }}")
        label = {
            "up": "Maju", "down": "Mundur",
            "left": "Kiri", "right": "Kanan", None: "Berhenti",
        }.get(direction, "–")
        self._add_alert("info", "Manual Control", label)

    # ══════════════════════════════════════════════════════════
    # ROBOT STATUS CARD
    # ══════════════════════════════════════════════════════════
    def _build_status_card(self):
        card, hlay, body = self._card_shell("Robot status")
        body.setStyleSheet("background:white; border:none;")

        grid = QGridLayout(body)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(0)
        grid.setVerticalSpacing(0)

        def _cell(icon_txt, label_txt, init_val="–", accent=None, is_bat=False):
            cell = QWidget()
            cell.setStyleSheet("background:white;")
            cly = QHBoxLayout(cell)
            cly.setContentsMargins(12, 0, 12, 0)
            cly.setSpacing(8)

            strip = QFrame()
            strip.setFixedWidth(5)
            strip.setMinimumHeight(28)
            strip.setStyleSheet(
                f"background:{accent if accent else PRIMARY}; border-radius:3px; border:none;")

            ic = QLabel(icon_txt)
            ic.setFixedWidth(20)
            ic.setStyleSheet("font-size:16px; background:transparent;")

            lb = QLabel(label_txt)
            lb.setStyleSheet("font-size:13px; color:#999; background:transparent;")

            cly.addWidget(strip, 0, Qt.AlignVCenter)
            cly.addWidget(ic,    0, Qt.AlignVCenter)
            cly.addWidget(lb,    0, Qt.AlignVCenter)
            cly.addStretch()

            if is_bat:
                bw = QWidget()
                bw.setStyleSheet("background:transparent;")
                bl = QHBoxLayout(bw)
                bl.setContentsMargins(0, 0, 0, 0)
                bl.setSpacing(6)
                self._bat_bar = QLabel()
                self._bat_bar.setFixedSize(56, 13)
                self._bat_bar.setPixmap(make_battery_pixmap(78, 56, 13))
                self._st_battery = QLabel("78%")
                self._st_battery.setStyleSheet(
                    "font-size:15px; font-weight:bold; color:#111; background:transparent;")
                bl.addWidget(self._bat_bar, 0, Qt.AlignVCenter)
                bl.addWidget(self._st_battery, 0, Qt.AlignVCenter)
                cly.addWidget(bw, 0, Qt.AlignVCenter)
                return cell, None
            else:
                val = QLabel(init_val)
                val.setStyleSheet(
                    "font-size:13px; font-weight:bold; color:#111; background:transparent;")
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)  # PyQt5: Qt.AlignRight
                cly.addWidget(val, 0, Qt.AlignVCenter)
                return cell, val

        c0, self._st_state   = _cell("🔄", "State",    "PATROL",            GREEN_OK)
        c1, _                = _cell("🔋", "Battery",  "",                  "#FFD600", is_bat=True)
        c2, self._st_speed   = _cell("⚡", "Speed",    "0.00 m/s",          PRIMARY)
        c3, self._st_mission = _cell("📍", "Mission",  "Heading to WP 1",   "#1565C0")
        c4, self._st_detect  = _cell("👤", "Detections","0 Persons",        "#2E7D32")
        c5, self._st_uptime  = _cell("⏱", "Uptime",   "00:00:00",          ORANGE)

        self._st_state.setStyleSheet(
            f"font-size:12px; font-weight:bold; color:{GREEN_OK}; background:transparent;")
        self._st_mission.setStyleSheet(
            "font-size:13px; font-weight:bold; color:#1565C0; background:transparent;")

        def _hdiv():
            d = QFrame()
            d.setFrameShape(QFrame.HLine)  # PyQt5: QFrame.HLine
            d.setFixedHeight(1)
            d.setStyleSheet("background:#EEEEEE; border:none;")
            return d

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)  # PyQt5: QFrame.VLine
        sep.setStyleSheet("background:#EEEEEE; border:none;")
        sep.setFixedWidth(1)

        grid.addWidget(c0,       0, 0)
        grid.addWidget(c1,       0, 2)
        grid.addWidget(_hdiv(),  1, 0)
        grid.addWidget(_hdiv(),  1, 2)
        grid.addWidget(c2,       2, 0)
        grid.addWidget(c3,       2, 2)
        grid.addWidget(_hdiv(),  3, 0)
        grid.addWidget(_hdiv(),  3, 2)
        grid.addWidget(c4,       4, 0)
        grid.addWidget(c5,       4, 2)
        grid.addWidget(sep,      0, 1, 5, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnMinimumWidth(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(2, 1)
        grid.setRowStretch(4, 1)

        return card

    # ══════════════════════════════════════════════════════════
    # ALERTS LOG CARD
    # ══════════════════════════════════════════════════════════
    def _build_alerts_card(self):
        card, hlay, body = self._card_shell("Alerts log")

        clr = QPushButton("Clear All")
        clr.setFixedSize(70, 24)
        clr.setStyleSheet(f"""
            QPushButton {{ background:#3B30B8; color:white; border-radius:5px;
                           font-size:11px; font-weight:bold; border:none; }}
            QPushButton:hover {{ background:#5548D0; }}
        """)
        clr.clicked.connect(self._clear_alerts)
        hlay.addWidget(clr)

        blay = QVBoxLayout(body)
        blay.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"""
            QScrollArea {{ border:none; background:{WHITE}; }}
            QScrollBar:vertical {{ width:6px; background:#F0F0F0;
                                   border-radius:3px; }}
            QScrollBar::handle:vertical {{ background:{PRIMARY};
                                           border-radius:3px; min-height:20px; }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {{ height:0px; }}
        """)

        self._alerts_container = QWidget()
        self._alerts_container.setStyleSheet(f"background:{WHITE};")
        self._alerts_inner = QVBoxLayout(self._alerts_container)
        self._alerts_inner.setContentsMargins(8, 8, 8, 8)
        self._alerts_inner.setSpacing(6)
        self._alerts_inner.addStretch()
        scroll.setWidget(self._alerts_container)

        self._alerts_scroll_area = scroll
        blay.addWidget(scroll)
        return card

    def _add_alert(self, kind, title, detail=""):
        ts  = datetime.now().strftime("%H:%M:%S")
        cfg = {
            "danger":  (RED_ALERT, "#FFF5F5"),
            "warning": (ORANGE,    "#FFFBF0"),
            "info":    ("#1565C0", "#F5F8FF"),
            "success": ("#2E7D32", "#F5FFF7"),
        }
        accent, bg = cfg.get(kind, cfg["info"])

        row = QFrame()
        row.setStyleSheet(f"background:{bg}; border-radius:8px; border:none;")
        rlay = QHBoxLayout(row)
        rlay.setContentsMargins(0, 0, 10, 0)
        rlay.setSpacing(10)

        strip = QFrame()
        strip.setFixedWidth(4)
        strip.setStyleSheet(f"background:{accent}; border-radius:2px; border:none;")
        rlay.addWidget(strip)

        tv = QVBoxLayout()
        tv.setSpacing(1)
        tv.setContentsMargins(0, 6, 0, 6)
        top = QHBoxLayout()
        top.setSpacing(6)
        tl  = QLabel(ts)
        tl.setStyleSheet(
            f"font-size:10px; color:{accent}; font-weight:bold; background:transparent;")
        ttl = QLabel(title)
        ttl.setStyleSheet(
            f"font-size:11px; font-weight:bold; color:{accent}; background:transparent;")
        top.addWidget(tl)
        top.addWidget(ttl)
        top.addStretch()
        tv.addLayout(top)

        if detail:
            dl = QLabel(detail)
            dl.setStyleSheet("font-size:10px; color:#555; background:transparent;")
            tv.addWidget(dl)

        rlay.addLayout(tv)

        self._alerts_inner.insertWidget(0, row)
        self._alert_rows.insert(0, row)
        while len(self._alert_rows) > 100:
            old = self._alert_rows.pop()
            old.deleteLater()

        QTimer.singleShot(30, lambda: (
            self._alerts_scroll_area.verticalScrollBar().setValue(0)
            if hasattr(self, '_alerts_scroll_area') else None))

    def _clear_alerts(self):
        for r in self._alert_rows:
            r.deleteLater()
        self._alert_rows.clear()

    # ══════════════════════════════════════════════════════════
    # SENSOR UPDATE
    # ══════════════════════════════════════════════════════════
    def _update_sensors(self):
        data     = self.arduino.generate()
        ultra    = data["ultra"]
        enc_spd  = data["enc_speed"]
        battery  = data["battery"]
        uptime_s = data["uptime"]

        self._st_speed.setText(f"{enc_spd} m/s")

        count_txt = (f"{self._person_count} "
                     f"Person{'s' if self._person_count != 1 else ''}")
        self._st_detect.setText(count_txt)
        pcol = RED_ALERT if self._person_count > 0 else "#111"
        self._st_detect.setStyleSheet(
            f"font-size:12px; font-weight:bold; color:{pcol}; background:transparent;")

        self._st_battery.setText(f"{battery:.0f}%")
        self._bat_bar.setPixmap(make_battery_pixmap(battery, 46, 10))

        h = uptime_s // 3600
        m = (uptime_s % 3600) // 60
        s = uptime_s % 60
        self._st_uptime.setText(f"{h:02d}:{m:02d}:{s:02d}")

        sc = GREEN_OK if self._robot_state == "PATROL" else RED_ALERT
        self._st_state.setText(self._robot_state)
        self._st_state.setStyleSheet(
            f"font-size:13px; font-weight:bold; color:{sc}; background:transparent;")

        if ultra < 30:
            kind = "danger" if ultra < 20 else "warning"
            self._add_alert(kind, "Obstacle Warning", f"Distance: {ultra} cm")
        if battery < 20:
            self._add_alert("danger", "Low Battery", f"Battery: {battery:.0f}%")

    # ══════════════════════════════════════════════════════════
    # KEYBOARD CONTROL
    # ══════════════════════════════════════════════════════════
    def keyPressEvent(self, event):
        km = {
            Qt.Key_W: "up",    # PyQt5: Qt.Key_W
            Qt.Key_S: "down",
            Qt.Key_A: "left",
            Qt.Key_D: "right",
        }
        if event.key() in km:
            self._manual_move(km[event.key()])
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            self._manual_move(None)

    # ══════════════════════════════════════════════════════════
    # RESIZE / CLOSE
    # ══════════════════════════════════════════════════════════
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.menu_drop.setFixedWidth(self.width())
        if hasattr(self, '_cam_overlay') and self._cam_overlay.isVisible():
            self._cam_overlay.setGeometry(
                0, 0, self.central.width(), self.central.height())

    def closeEvent(self, event):
        self.detector.stop()
        super().closeEvent(event)


# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    if "--disable-gpu" not in sys.argv:
        sys.argv += ["--disable-gpu", "--use-gl=swiftshader"]
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = SecurityRobotPyQt()
    win.show()

    # Seed alerts
    win._add_alert("success", "System Started",   "Robot online, patrol dimulai")
    win._add_alert("success", "Arrived at WP 1",  "Checkpoint reached — continuing patrol")
    win._add_alert("info",    "Heading to WP 2",  "Robot on patrol route")
    win._add_alert("warning", "Obstacle Warning", "Distance: 1.2 m")
    win._add_alert("danger",  "Person Detected",  "Confidence: 0.92")

    sys.exit(app.exec_())  # PyQt5: exec_() bukan exec()