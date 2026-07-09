/*
  =====================================================================
   LoRa Link Tester - TRANSMITTER
   Board  : ESP32 DevKit + Modul RFM95 (915 MHz)
   Fungsi : Kirim paket REQ berkala, tunggu ACK dari receiver (Mappi32),
            hitung RTT, RSSI, SNR, dan success rate, lalu kirim semua
            data itu lewat Serial (USB) dalam format CSV supaya bisa
            dibaca oleh GUI Python (lora_link_monitor_gui.py).

   WIRING RFM95 -> ESP32 (silakan sesuaikan kalau pengkabelanmu beda):
     RFM95 VIN  -> 3V3
     RFM95 GND  -> GND
     RFM95 SCK  -> GPIO18
     RFM95 MISO -> GPIO19
     RFM95 MOSI -> GPIO23
     RFM95 NSS  -> GPIO5
     RFM95 RST  -> GPIO14
     RFM95 DIO0 -> GPIO2

   Library yang dibutuhkan (Library Manager Arduino IDE):
     - "LoRa" by Sandeep Mistry
  =====================================================================
*/

#include <SPI.h>
#include <LoRa.h>

// ---------------- PIN CONFIG (ubah sesuai wiring kamu) ----------------
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2

// ---------------- RADIO CONFIG ----------------
#define LORA_FREQ      915E6   // 915 MHz (cek regulasi lokal, Indonesia umumnya 920-923 MHz)
#define LORA_SF        9       // Spreading Factor 9 (samain dengan RX!)
#define LORA_BW        125E3   // Bandwidth 125kHz
#define LORA_CR        5       // Coding rate 4/5
#define LORA_TXPOWER   17      // dBm (max 20 utk RFM95, disarankan 17-20)
#define LORA_SYNCWORD  0xF3    // sync word, HARUS SAMA dengan RX Mappi32

// Timeout ACK dihitung dari perkiraan airtime SF9 @125kHz utk paket kecil.
// SF9 airtime paket pendek (~20 byte) sekitar 100-150ms, kita kasih margin.
#define ACK_TIMEOUT_MS   400
#define SEND_INTERVAL_MS 1000  // jeda antar pengiriman REQ

uint32_t seq = 0;
uint32_t totalSent = 0;
uint32_t totalAck = 0;
uint32_t totalLost = 0;
int      lastRssi = 0;
float    lastSnr = 0;
uint32_t lastRtt = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LOG,LoRa init GAGAL. Cek wiring RFM95!");
    while (1) { delay(1000); }
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setTxPower(LORA_TXPOWER);

  Serial.println("LOG,TX siap. Mulai kirim REQ...");
}

void loop() {
  seq++;
  totalSent++;

  uint32_t t0 = millis();

  // Kirim paket REQ
  LoRa.beginPacket();
  LoRa.print("REQ:");
  LoRa.print(seq);
  LoRa.endPacket();

  // Tunggu ACK dengan timeout
  bool gotAck = false;
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String msg = "";
      while (LoRa.available()) {
        msg += (char)LoRa.read();
      }
      if (msg.startsWith("ACK:")) {
        uint32_t ackSeq = msg.substring(4).toInt();
        if (ackSeq == seq) {
          gotAck = true;
          lastRtt = millis() - t0;
          lastRssi = LoRa.packetRssi();
          lastSnr = LoRa.packetSnr();
          break;
        }
      }
    }
  }

  if (gotAck) {
    totalAck++;
  } else {
    totalLost++;
  }

  float successRate = (totalSent > 0) ? (100.0 * totalAck / totalSent) : 0.0;

  // Baris data untuk GUI (CSV): DATA,seq,totalSent,totalAck,totalLost,successRate,rssi,snr,rtt
  Serial.print("DATA,");
  Serial.print(seq); Serial.print(",");
  Serial.print(totalSent); Serial.print(",");
  Serial.print(totalAck); Serial.print(",");
  Serial.print(totalLost); Serial.print(",");
  Serial.print(successRate, 1); Serial.print(",");
  Serial.print(gotAck ? lastRssi : 0); Serial.print(",");
  Serial.print(gotAck ? lastSnr : 0, 1); Serial.print(",");
  Serial.println(gotAck ? lastRtt : 0);

  if (!gotAck) {
    Serial.print("LOG,Paket #");
    Serial.print(seq);
    Serial.println(" HILANG (tidak ada ACK)");
  }

  delay(SEND_INTERVAL_MS);
}
