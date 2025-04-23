#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

#define gpsSerial Serial2       // GPS on Serial2
#define LCD_ADDR 0x27           // Change if needed

TinyGPSPlus gps;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();       // IMU on default I2C (Wire)
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);          // LCD on same I2C bus
File logFile;

// Sensor values
float avx = 0, avy = 0, avz = 0;
float accx = 0, accy = 0, accz = 0;
float magx = 0, magy = 0, magz = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.print("Init SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    lcd.clear();
    lcd.print("SD FAIL");
    while (1);
  }

  logFile = SD.open("gps_log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Latitude,Longitude,Elevation,Satellites,AV_X,AV_Y,AV_Z,Acc_X,Acc_Y,Acc_Z,Mag_X,Mag_Y,Mag_Z");
    logFile.flush();
  }

  lcd.clear();
  lcd.print("Init IMU...");

  if (!lsm.begin()) {
    lcd.clear();
    lcd.print("IMU FAIL");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  lcd.clear();
  lcd.print("Waiting for GPS");
}

void loop() {
  // Read IMU
  lsm.read();
  accx = lsm.accelData.x;
  accy = lsm.accelData.y;
  accz = lsm.accelData.z;
  avx  = lsm.gyroData.x;
  avy  = lsm.gyroData.y;
  avz  = lsm.gyroData.z;
  magx = lsm.magData.x;
  magy = lsm.magData.y;
  magz = lsm.magData.z;

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      float alt = gps.altitude.isValid() ? gps.altitude.meters() : -1.0;
      int sats = gps.satellites.value();

      // Send to Serial
      Serial.print(lat, 6); Serial.print(",");
      Serial.print(lng, 6); Serial.print(",");
      Serial.print(alt, 2); Serial.print(",");
      Serial.print(sats); Serial.print(",");

      Serial.print(avx, 2); Serial.print(",");
      Serial.print(avy, 2); Serial.print(",");
      Serial.print(avz, 2); Serial.print(",");

      Serial.print(accx, 2); Serial.print(",");
      Serial.print(accy, 2); Serial.print(",");
      Serial.print(accz, 2); Serial.print(",");

      Serial.print(magx, 2); Serial.print(",");
      Serial.print(magy, 2); Serial.print(",");
      Serial.println(magz, 2);

      // Log to SD
      if (logFile) {
        logFile.print(lat, 6); logFile.print(",");
        logFile.print(lng, 6); logFile.print(",");
        logFile.print(alt, 2); logFile.print(",");
        logFile.print(sats); logFile.print(",");

        logFile.print(avx, 2); logFile.print(",");
        logFile.print(avy, 2); logFile.print(",");
        logFile.print(avz, 2); logFile.print(",");

        logFile.print(accx, 2); logFile.print(",");
        logFile.print(accy, 2); logFile.print(",");
        logFile.print(accz, 2); logFile.print(",");

        logFile.print(magx, 2); logFile.print(",");
        logFile.print(magy, 2); logFile.print(",");
        logFile.println(magz, 2);
        logFile.flush();
      }

      // LCD output (GPS only)
      lcd.setCursor(0, 0);
      lcd.print("N:"); lcd.print(abs(lat), 2); lcd.print((lat < 0) ? "S" : "N");
      lcd.setCursor(8, 0);
      lcd.print("E:"); lcd.print(abs(lng), 2); lcd.print((lng < 0) ? "W" : "E");

      lcd.setCursor(0, 1);
      lcd.print("Sats:     ");
      lcd.setCursor(6, 1);
      lcd.print(sats);
    }
  }
}
