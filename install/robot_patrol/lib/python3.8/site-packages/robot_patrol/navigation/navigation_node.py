# navigation/navigation_node.py

import random
import time

# Status gerak yang mungkin
GERAK_POSSIBLE = [
    "MAJU",
    "MUNDUR", 
    "BELOK_KIRI",
    "BELOK_KANAN",
    "BERHENTI"
]

class NavigasiRobot:
    def __init__(self):
        self.status_sekarang = "BERHENTI"
        self.kecepatan = 0  # cm/s
        self.posisi_x = 0.0
        self.posisi_y = 0.0
        self.heading = 0  # derajat (0 = Utara)

    def update_posisi(self):
        """
        Simulasi update posisi robot berdasarkan gerakannya.
        Di hardware asli, ini pakai data odometry / encoder motor.
        """
        if self.status_sekarang == "MAJU":
            self.posisi_y += round(self.kecepatan * 0.1, 2)
        elif self.status_sekarang == "MUNDUR":
            self.posisi_y -= round(self.kecepatan * 0.1, 2)
        elif self.status_sekarang == "BELOK_KIRI":
            self.heading = (self.heading - 10) % 360
        elif self.status_sekarang == "BELOK_KANAN":
            self.heading = (self.heading + 10) % 360

    def terima_perintah(self, perintah: str):
        """
        Simulasi menerima perintah gerak.
        Di hardware asli, ini akan subscribe ke ROS topic /cmd_vel.
        """
        if perintah in GERAK_POSSIBLE:
            self.status_sekarang = perintah
            if perintah == "BERHENTI":
                self.kecepatan = 0
            else:
                self.kecepatan = random.randint(10, 50)

    def jalankan_navigasi(self):
        print("[NAVIGASI] Sistem navigasi mulai berjalan...")
        print("[NAVIGASI] Robot siap menerima perintah...\n")

        while True:
            # Simulasi terima perintah random
            perintah = random.choice(GERAK_POSSIBLE)
            self.terima_perintah(perintah)
            self.update_posisi()

            # Tentukan emoji status
            emoji = {
                "MAJU": "⬆️",
                "MUNDUR": "⬇️",
                "BELOK_KIRI": "⬅️",
                "BELOK_KANAN": "➡️",
                "BERHENTI": "⏹️"
            }

            print(f"[NAVIGASI] Perintah  : {emoji[self.status_sekarang]} {self.status_sekarang}")
            print(f"[NAVIGASI] Kecepatan : {self.kecepatan} cm/s")
            print(f"[NAVIGASI] Posisi    : X={self.posisi_x:.1f} | Y={self.posisi_y:.1f}")
            print(f"[NAVIGASI] Heading   : {self.heading}°")
            print()

            time.sleep(1.5)

if __name__ == "__main__":
    robot = NavigasiRobot()
    robot.jalankan_navigasi()