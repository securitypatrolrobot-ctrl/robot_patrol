import asyncio
import websockets
import json
import threading
import time
import random
import sys
import os

sys.path.append(os.path.dirname(__file__))

from failsafe.lidar_node import baca_lidar, cek_rintangan
from vision.detector import deteksi_objek
from communication.lora_handler import simulasi_kirim_lora, simulasi_terima_lora
from navigation.navigation_node import NavigasiRobot

state = {
    "ultrasonik_cm"   : 0.0,
    "lidar_data"      : {},
    "objek_terdeteksi": None,
    "confidence"      : 0.0,
    "navigasi_status" : "BERHENTI",
    "kecepatan"       : 0,
    "battery"         : 100.0,
    "lora_status"     : "OK",
    "failsafe_aktif"  : False,
    "posisi_x"        : 0.0,
    "posisi_y"        : 0.0,
    "heading"         : 0,
    "perintah_lora"   : None,
}

robot = NavigasiRobot()

def thread_ultrasonik():
    while True:
        state["ultrasonik_cm"] = round(random.uniform(5.0, 300.0), 2)
        time.sleep(1)

def thread_lidar():
    while True:
        state["lidar_data"] = baca_lidar()
        time.sleep(2)

def thread_vision():
    while True:
        objek, conf = deteksi_objek()
        state["objek_terdeteksi"] = objek
        state["confidence"] = round(conf, 2)
        time.sleep(1.5)

def thread_lora():
    while True:
        data_kirim = {
            "robot_id": "PATROL-01",
            "battery" : state["battery"],
            "status"  : state["navigasi_status"],
            "failsafe": state["failsafe_aktif"],
        }
        berhasil, _ = simulasi_kirim_lora(data_kirim)
        state["lora_status"] = "OK" if berhasil else "GAGAL"
        perintah = simulasi_terima_lora()
        if perintah:
            state["perintah_lora"] = perintah
        time.sleep(3)

def thread_navigasi():
    while True:
        if state["failsafe_aktif"]:
            robot.terima_perintah("BERHENTI")
        else:
            robot.terima_perintah(random.choice(["MAJU", "BELOK_KIRI", "BELOK_KANAN"]))
        robot.update_posisi()
        state["navigasi_status"] = robot.status_sekarang
        state["kecepatan"]       = robot.kecepatan
        state["posisi_x"]        = round(robot.posisi_x, 2)
        state["posisi_y"]        = round(robot.posisi_y, 2)
        state["heading"]         = robot.heading
        time.sleep(1.5)

def thread_failsafe():
    while True:
        bahaya = state["ultrasonik_cm"] < 30.0
        if state["lidar_data"]:
            ada, _ = cek_rintangan(state["lidar_data"])
            if ada:
                bahaya = True
        state["failsafe_aktif"] = bahaya
        time.sleep(0.5)

def thread_battery():
    while True:
        if state["navigasi_status"] != "BERHENTI":
            state["battery"] = max(0.0, round(state["battery"] - 0.01, 2))
        time.sleep(1)

async def handler(websocket):
    print(f"[WS] Client konek: {websocket.remote_address}")
    try:
        while True:
            lidar_all = {str(k): v for k, v in state["lidar_data"].items()}
            payload = {
                "ultrasonik_cm"   : state["ultrasonik_cm"],
                "lidar_depan"     : state["lidar_data"].get(0, None),
                "lidar_all"       : lidar_all,
                "objek_terdeteksi": state["objek_terdeteksi"],
                "confidence"      : state["confidence"],
                "navigasi_status" : state["navigasi_status"],
                "kecepatan"       : state["kecepatan"],
                "battery"         : round(state["battery"], 1),
                "lora_status"     : state["lora_status"],
                "failsafe_aktif"  : state["failsafe_aktif"],
                "posisi_x"        : state["posisi_x"],
                "posisi_y"        : state["posisi_y"],
                "heading"         : state["heading"],
                "perintah_lora"   : state["perintah_lora"],
                "timestamp"       : time.strftime("%H:%M:%S"),
            }
            await websocket.send(json.dumps(payload))
            await asyncio.sleep(1.5)
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] Client disconnect")

async def main():
    print("=" * 45)
    print("  Robot Patrol - WebSocket Server")
    print("=" * 45)
    print("  ws://0.0.0.0:8765")
    print("=" * 45)
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    for t in [thread_ultrasonik, thread_lidar, thread_vision,
              thread_lora, thread_navigasi, thread_failsafe, thread_battery]:
        threading.Thread(target=t, daemon=True).start()
    time.sleep(1)
    asyncio.run(main())
