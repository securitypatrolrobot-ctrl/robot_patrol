# vision/detector.py

import random
import time

# Daftar objek yang mungkin terdeteksi
OBJEK_POSSIBLE = [
    "person",
    "car",
    "bicycle",
    "dog",
    "unknown_object",
    None  # Tidak ada objek terdeteksi
]

def deteksi_objek():
    """
    Simulasi deteksi objek dari kamera.
    Di hardware asli, ini akan pakai model AI (YOLO, dll)
    yang membaca frame dari kamera secara real-time.
    """
    objek = random.choice(OBJEK_POSSIBLE)
    
    if objek is None:
        return None, 0.0
    
    # Simulasi confidence score (seberapa yakin AI mendeteksi objek)
    confidence = round(random.uniform(0.60, 0.99), 2)
    return objek, confidence

def jalankan_vision():
    print("[VISION] Kamera mulai berjalan...")
    print("[VISION] Mendeteksi objek...\n")

    while True:
        objek, confidence = deteksi_objek()

        if objek is None:
            print("[VISION] Tidak ada objek terdeteksi")
        else:
            # Tentukan level ancaman
            if objek == "person":
                level = "🔴 PRIORITAS TINGGI"
            elif objek in ["car", "bicycle"]:
                level = "🟡 PERLU DIPERHATIKAN"
            else:
                level = "🟢 NORMAL"

            print(f"[VISION] Objek: {objek:<15} | "
                  f"Confidence: {confidence} | "
                  f"Level: {level}")

        time.sleep(1.5)  # Deteksi setiap 1.5 detik

if __name__ == "__main__":
    jalankan_vision()