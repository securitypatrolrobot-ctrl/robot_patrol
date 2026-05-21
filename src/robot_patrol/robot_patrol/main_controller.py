# main_controller.py
# Otak utama robot patrol - menggabungkan semua subsistem

import threading
import time
import random

# ============================================================
# IMPORT semua subsistem
# ============================================================
import sys
import os

sys.path.append(os.path.dirname(__file__))

from failsafe.ultrasonic_node import baca_jarak
from failsafe.lidar_node import baca_lidar, cek_rintangan
from vision.detector import deteksi_objek
from communication.lora_handler import simulasi_kirim_lora, simulasi_terima_lora
from navigation.navigation_node import NavigasiRobot

# ============================================================
# STATE GLOBAL robot (data terbaru dari semua subsistem)
# ============================================================
state = {
    "ultrasonik_cm"   : 0,
    "lidar_data"      : {},
    "objek_terdeteksi": None,
    "confidence"      : 0.0,
    "navigasi_status" : "BERHENTI",
    "battery"         : 100,
    "lora_status"     : "OK",
    "failsafe_aktif"  : False,
}

robot = NavigasiRobot()

# ============================================================
# THREAD setiap subsistem (jalan bersamaan)
# ============================================================

def thread_ultrasonik():
    while True:
        state["ultrasonik_cm"] = baca_jarak()
        time.sleep(1)

def thread_lidar():
    while True:
        data = baca_lidar()
        state["lidar_data"] = data
        time.sleep(2)

def thread_vision():
    while True:
        objek, conf = deteksi_objek()
        state["objek_terdeteksi"] = objek
        state["confidence"] = conf
        time.sleep(1.5)

def thread_lora():
    while True:
        data_kirim = {
            "robot_id" : "PATROL-01",
            "battery"  : state["battery"],
            "status"   : state["navigasi_status"],
            "failsafe" : state["failsafe_aktif"],
        }
        berhasil, _ = simulasi_kirim_lora(data_kirim)
        state["lora_status"] = "OK" if berhasil else "GAGAL"

        perintah = simulasi_terima_lora()
        if perintah:
            robot.terima_perintah(perintah)
        time.sleep(3)

def thread_navigasi():
    while True:
        # Kalau failsafe aktif, robot berhenti
        if state["failsafe_aktif"]:
            robot.terima_perintah("BERHENTI")
        else:
            perintah = random.choice(["MAJU", "BELOK_KIRI", "BELOK_KANAN"])
            robot.terima_perintah(perintah)

        robot.update_posisi()
        state["navigasi_status"] = robot.status_sekarang
        time.sleep(1.5)

def thread_failsafe():
    """
    Cek kondisi berbahaya dari semua sensor.
    Kalau ada bahaya, aktifkan failsafe → robot berhenti.
    """
    while True:
        bahaya = False

        # Cek ultrasonik
        if state["ultrasonik_cm"] < 30:
            bahaya = True

        # Cek lidar
        if state["lidar_data"]:
            ada_rintangan, _ = cek_rintangan(state["lidar_data"])
            if ada_rintangan:
                bahaya = True

        state["failsafe_aktif"] = bahaya
        time.sleep(0.5)

# ============================================================
# DASHBOARD - tampilkan semua status di terminal
# ============================================================

def tampilkan_dashboard():
    while True:
        os.system("clear")  # Bersihkan terminal

        print("=" * 55)
        print("        🤖 ROBOT PATROL - MAIN CONTROLLER")
        print("=" * 55)

        # Failsafe
        if state["failsafe_aktif"]:
            print("  🚨 FAILSAFE AKTIF - ROBOT BERHENTI DARURAT!")
        else:
            print("  ✅ Sistem Normal")

        print("-" * 55)

        # Sensor
        print("  📡 SENSOR")
        print(f"    Ultrasonik : {state['ultrasonik_cm']} cm")
        lidar_depan = state['lidar_data'].get(0, '-')
        print(f"    LiDAR depan: {lidar_depan} m")

        print("-" * 55)

        # Vision
        print("  🎥 VISION")
        if state["objek_terdeteksi"]:
            print(f"    Objek      : {state['objek_terdeteksi']}")
            print(f"    Confidence : {state['confidence']}")
        else:
            print("    Tidak ada objek terdeteksi")

        print("-" * 55)

        # Navigasi
        print("  🧭 NAVIGASI")
        print(f"    Status     : {state['navigasi_status']}")
        print(f"    Posisi     : X={robot.posisi_x:.1f} | Y={robot.posisi_y:.1f}")
        print(f"    Heading    : {robot.heading}°")

        print("-" * 55)

        # Komunikasi & Battery
        print("  📶 KOMUNIKASI & DAYA")
        print(f"    LoRa       : {state['lora_status']}")
        print(f"    Battery    : {state['battery']}%")

        print("=" * 55)
        print("  Tekan CTRL+C untuk berhenti")
        print("=" * 55)

        time.sleep(1)

# ============================================================
# MAIN - jalankan semua thread
# ============================================================

if __name__ == "__main__":
    print("🤖 Menginisialisasi Robot Patrol...")
    time.sleep(1)

    threads = [
        threading.Thread(target=thread_ultrasonik, daemon=True),
        threading.Thread(target=thread_lidar,      daemon=True),
        threading.Thread(target=thread_vision,     daemon=True),
        threading.Thread(target=thread_lora,       daemon=True),
        threading.Thread(target=thread_navigasi,   daemon=True),
        threading.Thread(target=thread_failsafe,   daemon=True),
    ]

    for t in threads:
        t.start()

    print("✅ Semua subsistem berjalan!")
    time.sleep(1)

    tampilkan_dashboard()