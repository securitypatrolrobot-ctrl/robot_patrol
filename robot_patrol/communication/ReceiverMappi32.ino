/*
  =====================================================================
   LoRa Link Tester - RECEIVER
   Board  : Mappi32 by KMTek (Yogyakarta - Indonesia)
   Fungsi : Terima paket REQ dari transmitter (ESP32+RFM95), langsung
            balas ACK dengan nomor seq yang sama supaya TX bisa hitung
            RTT & RSSI. Tanpa sensor apapun.

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
// SPI pins: SCK, MISO, MOSI (SS dipakai ulang dari define di atas)
#define LORA_SCK   14
#define LORA_MISO  12
#define LORA_MOSI  13

// ---------------- RADIO CONFIG (harus SAMA PERSIS dengan TX) ----------------
#define LORA_FREQ      915E6
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_TXPOWER   17
#define LORA_SYNCWORD  0xF3   // samakan dengan TX

uint32_t packetCount = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("LoRa Receiver (Mappi32)");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, ss);
  LoRa.setPins(ss, rst, dio0);

  while (!LoRa.begin(LORA_FREQ)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setTxPower(LORA_TXPOWER);

  Serial.println("LoRa Initializing OK! Menunggu paket REQ...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String msg = "";
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (msg.startsWith("REQ:")) {
    String seqStr = msg.substring(4);
    packetCount++;

    Serial.print("Terima REQ #");
    Serial.print(seqStr);
    Serial.print(" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.println(snr);

    // Balas ACK secepatnya
    LoRa.beginPacket();
    LoRa.print("ACK:");
    LoRa.print(seqStr);
    LoRa.endPacket();
  }
}
