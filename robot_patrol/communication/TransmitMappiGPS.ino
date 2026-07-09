/*
  =====================================================================
   LoRa Robot Telemetry - TRANSMITTER (di ROBOT)
   Board  : Arduino Mega 2560
   Modul  : RFM9x (LoRa 915MHz, SPI) + GPS M10G (UBX config, UART) +
            QMC5883L (Compass, I2C) + MPU6050 (IMU, I2C, library
            Jeff Rowberg / Electronic Cats - SAMA seperti program
            "GPS + Compass + IMU" kamu, bukan library Adafruit)

   Sketch ini = penggabungan:
     - Logic baca sensor dari program "GPS + Compass + IMU" (GPS UBX
       config, QMC5883L compass, MPU6050 raw + kalibrasi gyro Z)
     - Logic kirim LoRa dari sketch TX sebelumnya (RFM9x, RST=10,
       DIO0=9, SS=53)

   Bedanya dengan versi lama: heading dari GPS course diganti heading
   dari COMPASS (QMC5883L), karena itu yang dipakai di program
   "paling kanan" itu. GPS cuma dipakai buat lat/lon (cog & speed
   di source aslinya memang di-comment, jadi tidak dikirim).

   Format paket LoRa (dikirim ke base station):
     NAV,seq,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt

   ---------------------------------------------------------------
   WIRING:

   RFM9x -> Arduino Mega (SPI hardware Mega: SCK=52, MISO=50, MOSI=51,
            pin ini FIXED; hanya CS/RST/DIO0 yang bebas dipilih)
     RFM9x VIN  -> 3V3
     RFM9x GND  -> GND
     RFM9x SCK  -> 52
     RFM9x MISO -> 50
     RFM9x MOSI -> 51
     RFM9x NSS  -> 53
     RFM9x RST  -> 10
     RFM9x DIO0 -> 9

   GPS M10G -> Arduino Mega
     GPS TX -> Mega RX1 (pin 19)
     GPS RX -> Mega TX1 (pin 18)

   QMC5883L (Compass) + MPU6050 (IMU) -> satu bus I2C Mega
     SDA -> pin 20
     SCL -> pin 21
  =====================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>

// ---------------- LoRa pin & radio config ----------------
#define LORA_SS   53
#define LORA_RST  10
#define LORA_DIO0 9

#define LORA_FREQ      915E6
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_TXPOWER   17
#define LORA_SYNCWORD  0xF3

// interval kirim paket lewat LoRa (ms). Jangan terlalu cepat karena
// airtime SF9 lumayan lama untuk paket sepanjang ini.
#define SEND_INTERVAL_MS 500

TinyGPSPlus gps;
QMC5883LCompass compass;
MPU6050 mpu;

float gyro_z_offset = 0.0;
unsigned long last_time  = 0;
unsigned long lastSendMs = 0;
uint32_t seq = 0;

// ---------------- UBX helper (config GPS M10G) ----------------
void sendUBX(HardwareSerial &port, uint8_t cls, uint8_t id,
             const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  port.write(0xB5);
  port.write(0x62);
  auto ck = [&](uint8_t b) {
    port.write(b); ckA += b; ckB += ckA;
  };
  ck(cls); ck(id);
  ck(len & 0xFF); ck(len >> 8);
  for (uint16_t i = 0; i < len; i++) ck(payload[i]);
  port.write(ckA);
  port.write(ckB);
}

void configureGPS() {
  Serial1.begin(115200);
  delay(100);

  uint8_t cfgPrt[20] = {
    0x01, 0x00, 0x00, 0x00,
    0xC0, 0x08, 0x00, 0x00,
    0x80, 0x25, 0x00, 0x00,  // 9600 baud
    0x01, 0x00,
    0x02, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  sendUBX(Serial1, 0x06, 0x00, cfgPrt, sizeof(cfgPrt));
  Serial1.flush();
  delay(500);
  Serial1.end();
  delay(100);
  Serial1.begin(9600);
  delay(100);

  uint8_t enGGA[8] = {0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  sendUBX(Serial1, 0x06, 0x01, enGGA, sizeof(enGGA));
  delay(50);

  uint8_t enRMC[8] = {0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  sendUBX(Serial1, 0x06, 0x01, enRMC, sizeof(enRMC));
  delay(50);

  uint8_t disGSV[8] = {0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendUBX(Serial1, 0x06, 0x01, disGSV, sizeof(disGSV));
  delay(50);

  uint8_t cfgRate[6] = {
    0xC8, 0x00,  // measRate = 200ms -> 5Hz
    0x01, 0x00,
    0x01, 0x00
  };
  sendUBX(Serial1, 0x06, 0x08, cfgRate, sizeof(cfgRate));
  delay(50);

  Serial.println(F("[OK] GPS: NMEA, 9600 baud, 5Hz"));
}

void calibrateGyro() {
  Serial.println(F("Kalibrasi gyro, JANGAN DIGERAKIN dulu..."));
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(5);
  }
  gyro_z_offset = sum / 500.0;
  Serial.println(F("Kalibrasi gyro selesai."));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== LoRa TX (ROBOT) - GPS + Compass + IMU ==="));

  // ---- Init LoRa ----
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa init gagal, coba lagi..."));
    delay(500);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setTxPower(LORA_TXPOWER);
  Serial.println(F("LoRa OK."));

  // ---- Init GPS ----
  configureGPS();

  // ---- Init I2C, Compass, IMU ----
  Wire.begin();

  compass.init();
  // TODO: ganti angka kalibrasi ini sesuai hasil kalibrasi compass kamu sendiri
  compass.setCalibrationOffsets(1260.00, -634.00, -226.00);
  compass.setCalibrationScales(0.72, 0.74, 3.84);
  Serial.println(F("[OK] Compass siap"));

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("[OK] MPU6050 terdeteksi"));
  } else {
    Serial.println(F("[ERR] MPU6050 tidak terdeteksi! Cek wiring SDA/SCL."));
  }

  calibrateGyro();
  last_time  = millis();
  lastSendMs = millis();

  Serial.println(F("Siap kirim telemetry ke base station..."));
}

void loop() {
  // ---- Baca GPS terus-menerus ----
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // ---- Baca compass ----
  compass.read();
  int mx = compass.getX();
  int my = compass.getY();
  int mz = compass.getZ();
  int heading_raw_val = compass.getAzimuth();
  int compass_offset  = -13;
  float heading = (360 - heading_raw_val + compass_offset + 360) % 360;

  // ---- Baca IMU ----
  int16_t gx_r, gy_r, gz_r;
  int16_t ax_r, ay_r, az_r;
  mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

  float gx_degs = gx_r / 131.0;
  float gy_degs = gy_r / 131.0;
  float gz_degs = (gz_r - gyro_z_offset) / 131.0;

  float ax_ms2 = ax_r / 16384.0 * 9.81;
  float ay_ms2 = ay_r / 16384.0 * 9.81;
  float az_ms2 = az_r / 16384.0 * 9.81;

  unsigned long now_ms = millis();
  unsigned long dt_ms  = now_ms - last_time;
  last_time = now_ms;

  float lat = 0.0, lon = 0.0;
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  // ---- Kirim lewat LoRa tiap SEND_INTERVAL_MS ----
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    seq++;

    // NAV,seq,lat,lon,mx,my,mz,heading,gx,gy,gz,ax,ay,az,dt
    String pkt = "NAV,";
    pkt += seq;                    pkt += ",";
    pkt += String(lat, 6);         pkt += ",";
    pkt += String(lon, 6);         pkt += ",";
    pkt += mx;                     pkt += ",";
    pkt += my;                     pkt += ",";
    pkt += mz;                     pkt += ",";
    pkt += String(heading, 1);     pkt += ",";
    pkt += String(gx_degs, 2);     pkt += ",";
    pkt += String(gy_degs, 2);     pkt += ",";
    pkt += String(gz_degs, 2);     pkt += ",";
    pkt += String(ax_ms2, 2);      pkt += ",";
    pkt += String(ay_ms2, 2);      pkt += ",";
    pkt += String(az_ms2, 2);      pkt += ",";
    pkt += dt_ms;

    LoRa.beginPacket();
    LoRa.print(pkt);
    LoRa.endPacket();

    Serial.print(F("TX -> "));
    Serial.println(pkt);
  }
}
