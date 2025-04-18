#include <TinyGPSPlus.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>

#define gpsSerial Serial2
#define BUILTIN_SDCARD 254  // For Teensy's internal SD slot

TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Placeholder IMU values
float avx = 0.0, avy = 0.0, avz = 0.0;
float accx = 0.0, accy = 0.0, accz = 0.0;
float magx = 0.0, magy = 0.0, magz = 0.0;

File logFile;

void setup() {
  Serial.begin(9600);        // For GUI via USB
  gpsSerial.begin(9600);       // GPS on Serial2
  lcd.init();
  lcd.backlight();
  lcd.print("Init SD card...");

  // Initialize built-in SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    lcd.clear();
    lcd.print("SD FAIL");
    Serial.println("SD card init failed!");
    while (1);  // Halt if SD fails
  }

  // Open log file
  logFile = SD.open("gps_log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Latitude,Longitude,Elevation,Satellites,AV_X,AV_Y,AV_Z,Acc_X,Acc_Y,Acc_Z,Mag_X,Mag_Y,Mag_Z");
    logFile.flush();
  }

  lcd.clear();
  lcd.print("Waiting for GPS");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      float alt = gps.altitude.meters();
      int sats = gps.satellites.value();

      // Send to GUI
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
      Serial.print(magy, 2); Serial.println(magz, 2);

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
        logFile.print(magy, 2); logFile.println(magz, 2);
        logFile.flush();
      }

      // LCD (GPS only)
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
