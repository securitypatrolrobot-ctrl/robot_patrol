/*
  =====================================================================
   LoRa Robot Telemetry - RECEIVER (di BASE STATION)
   Board  : Mappi32 by KMTek (Yogyakarta - Indonesia)
   Fungsi : Terima paket NAV (GPS + Compass + IMU) dari robot (Mega
            TX), ukur RSSI/SNR link quality, lalu forward semuanya
            lewat Serial USB ke GUI Python
            (gui_lora_nav_monitor.py) yang jalan di laptop base
            station ini.

   Format paket MASUK dari robot (lewat LoRa):
     NAV,seq,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt

   Format output KE GUI (lewat Serial USB, HARUS SAMA PERSIS dengan
   yang diharapkan gui_lora_nav_monitor.py):
     DATA,seq,rssi,snr,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt,packetCount

   Pin sesuai contoh resmi Mappi32 by KMTek:
     SPI.begin(SCK=14, MISO=12, MOSI=13, SS=15)
     ss   = 15
     rst  = 0
     dio0 = 27

   Library yang dibutuhkan (Library Manager Arduino IDE):
     - "LoRa" by Sandeep Mistry
  =====================================================================
*/

#include <SPI.h>
#include <LoRa.h>

// ---------------- PIN CONFIG (Mappi32 by KMTek) ----------------
#define ss    15
#define rst   0
#define dio0  27
#define LORA_SCK   14
#define LORA_MISO  12
#define LORA_MOSI  13

// ---------------- RADIO CONFIG (harus SAMA PERSIS dengan TX) ----------------
#define LORA_FREQ      915E6
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_SYNCWORD  0xF3

uint32_t packetCount = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== LoRa RX (BASE STATION) - menunggu robot ===");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, ss);
  LoRa.setPins(ss, rst, dio0);

  while (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init gagal, coba lagi...");
    delay(500);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);

  Serial.println("LoRa OK. Siap menerima telemetry robot...");
}

// Pecah string "NAV,seq,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt"
// jadi array of String berdasarkan koma. Return jumlah field.
int splitCSV(const String &s, String out[], int maxFields) {
  int count = 0;
  int start = 0;
  while (count < maxFields) {
    int idx = s.indexOf(',', start);
    if (idx == -1) {
      out[count++] = s.substring(start);
      break;
    }
    out[count++] = s.substring(start, idx);
    start = idx + 1;
  }
  return count;
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String msg = "";
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }

    int rssi   = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();

    if (msg.startsWith("NAV,")) {
      packetCount++;

      // Format masuk: NAV,seq,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt
      String fields[15];
      int n = splitCSV(msg, fields, 15);

      if (n >= 15) {
        String seq     = fields[1];
        String lat     = fields[2];
        String lon     = fields[3];
        String mx      = fields[4];
        String my      = fields[5];
        String mz      = fields[6];
        String heading = fields[7];
        String gx      = fields[8];
        String gy      = fields[9];
        String gz      = fields[10];
        String ax      = fields[11];
        String ay      = fields[12];
        String az      = fields[13];
        String dt      = fields[14];

        // Format keluar ke GUI:
        // DATA,seq,rssi,snr,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt,packetCount
        Serial.print("DATA,");
        Serial.print(seq);      Serial.print(",");
        Serial.print(rssi);     Serial.print(",");
        Serial.print(snr, 1);   Serial.print(",");
        Serial.print(lat);      Serial.print(",");
        Serial.print(lon);      Serial.print(",");
        Serial.print(mx);       Serial.print(",");
        Serial.print(my);       Serial.print(",");
        Serial.print(mz);       Serial.print(",");
        Serial.print(heading);  Serial.print(",");
        Serial.print(gx);       Serial.print(",");
        Serial.print(gy);       Serial.print(",");
        Serial.print(gz);       Serial.print(",");
        Serial.print(ax);       Serial.print(",");
        Serial.print(ay);       Serial.print(",");
        Serial.print(az);       Serial.print(",");
        Serial.print(dt);       Serial.print(",");
        Serial.println(packetCount);
      } else {
        Serial.print("LOG,paket NAV format gak lengkap: ");
        Serial.println(msg);
      }
    } else {
      // paket lain yang bukan format NAV (misal beacon lama, dsb)
      Serial.print("LOG,paket asing diterima: ");
      Serial.println(msg);
    }
  }
}