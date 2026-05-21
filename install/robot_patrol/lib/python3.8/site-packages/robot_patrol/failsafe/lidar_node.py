# sensors/lidar/lidar_node.py

import random
import time

def baca_lidar():
    """
    Simulasi pembacaan LiDAR 360 derajat.
    Menghasilkan 360 titik jarak (1 titik per derajat).
    Di hardware asli, ini akan baca data dari driver LiDAR via serial/USB.
    """
    data = {}
    for sudut in range(0, 360, 45):  # Setiap 45 derajat (8 titik)
        jarak_m = round(random.uniform(0.2, 10.0), 2)
        data[sudut] = jarak_m
    return data

def cek_rintangan(data_lidar):
    """
    Cek apakah ada rintangan di depan robot (sudut 0, 315, 45 derajat).
    """
    sudut_depan = [0, 45, 315]
    for sudut in sudut_depan:
        if sudut in data_lidar and data_lidar[sudut] < 1.0:
            return True, data_lidar[sudut]
    return False, None

def jalankan_lidar():
    print("[LIDAR] Sensor mulai berjalan...")

    while True:
        data = baca_lidar()

        print("\n[LIDAR] Data jarak per arah:")
        arah = {0: "Depan", 45: "Depan-Kanan", 90: "Kanan",
                135: "Belakang-Kanan", 180: "Belakang",
                225: "Belakang-Kiri", 270: "Kiri", 315: "Depan-Kiri"}
        
        for sudut, jarak in data.items():
            print(f"  {arah[sudut]:<20} ({sudut}°): {jarak} m")

        ada_rintangan, jarak_rintangan = cek_rintangan(data)
        if ada_rintangan:
            print(f"  ⚠️  RINTANGAN DI DEPAN! Jarak: {jarak_rintangan} m")
        else:
            print(f"  ✅ Jalur depan AMAN")

        time.sleep(2)  # Scan setiap 2 detik

if __name__ == "__main__":
    jalankan_lidar()