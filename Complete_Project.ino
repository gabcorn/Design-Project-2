#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

// Serial interfaces
#define gpsSerial Serial2
#define btSerial  Serial1

// LCD config
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// IMU
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// GPS
TinyGPSPlus gps;

// SD
File logFile;

// Sensor values
float avx, avy, avz;
float accx, accy, accz;
float magx, magy, magz;

void setup() {
  Serial.begin(9600);      // USB for GUI/debug
  btSerial.begin(9600);    // Bluetooth output
  gpsSerial.begin(9600);   // GPS input

  lcd.init();
  lcd.backlight();
  lcd.print("Init SD card...");

  // SD card
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

  // LSM9DS1 IMU over I2C (Wire)
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
  // IMU read
  lsm.read();
  accx = lsm.accelData.x;
  accy = lsm.accelData.y;
  accz = lsm.accelData.z;

  avx = lsm.gyroData.x;
  avy = lsm.gyroData.y;
  avz = lsm.gyroData.z;

  magx = lsm.magData.x;
  magy = lsm.magData.y;
  magz = lsm.magData.z;

  // GPS read
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      float alt = gps.altitude.isValid() ? gps.altitude.meters() : -1.0;
      int sats = gps.satellites.value();

      // Combine into a single line
      String line =
        String(lat, 6) + "," + String(lng, 6) + "," + String(alt, 2) + "," + String(sats) + "," +
        String(avx, 2) + "," + String(avy, 2) + "," + String(avz, 2) + "," +
        String(accx, 2) + "," + String(accy, 2) + "," + String(accz, 2) + "," +
        String(magx, 2) + "," + String(magy, 2) + "," + String(magz, 2);

      // Output to USB + Bluetooth
      Serial.println(line);
      btSerial.println(line);

      // Output to SD
      if (logFile) {
        logFile.println(line);
        logFile.flush();
      }

      // Output to LCD (GPS only)
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
