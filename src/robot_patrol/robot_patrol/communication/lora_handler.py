# communication/lora_handler.py

import random
import time
import json

def simulasi_kirim_lora(data: dict):
    """
    Simulasi pengiriman data via LoRa.
    Di hardware asli, ini akan kirim data via serial port
    ke modul LoRa (misal: SX1276, E32, dll).
    """
    # Simulasi delay pengiriman (LoRa agak lambat)
    delay = round(random.uniform(0.5, 2.0), 2)
    time.sleep(delay)

    # Simulasi kemungkinan gagal kirim (5% chance)
    gagal = random.random() < 0.05

    if gagal:
        return False, "Sinyal lemah / timeout"
    return True, f"Terkirim dalam {delay} detik"

def simulasi_terima_lora():
    """
    Simulasi menerima perintah dari base station via LoRa.
    """
    perintah_possible = [
        "PATROL_START",
        "PATROL_STOP",
        "RETURN_TO_BASE",
        "STATUS_REQUEST",
        None  # Tidak ada perintah masuk
    ]
    return random.choice(perintah_possible)

def jalankan_lora():
    print("[LoRa] Modul komunikasi mulai berjalan...")
    print("[LoRa] Menunggu koneksi ke base station...\n")

    siklus = 1

    while True:
        print(f"--- Siklus #{siklus} ---")

        # Simulasi data status robot yang akan dikirim
        data_kirim = {
            "robot_id": "PATROL-01",
            "timestamp": time.strftime("%H:%M:%S"),
            "battery": random.randint(20, 100),
            "status": random.choice(["patrolling", "idle", "charging"]),
            "posisi": {
                "x": round(random.uniform(0, 50), 1),
                "y": round(random.uniform(0, 50), 1)
            }
        }

        # Kirim data
        print(f"[LoRa] Mengirim data: {json.dumps(data_kirim, indent=2)}")
        berhasil, pesan = simulasi_kirim_lora(data_kirim)

        if berhasil:
            print(f"[LoRa] ✅ {pesan}")
        else:
            print(f"[LoRa] ❌ Gagal kirim! Alasan: {pesan}")

        # Cek perintah masuk
        perintah = simulasi_terima_lora()
        if perintah:
            print(f"[LoRa] 📩 Perintah diterima: {perintah}")
        else:
            print(f"[LoRa] 📭 Tidak ada perintah masuk")

        siklus += 1
        print()
        time.sleep(3)

if __name__ == "__main__":
    jalankan_lora()