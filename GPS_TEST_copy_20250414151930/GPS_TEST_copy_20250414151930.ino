#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial ss(7, 8);
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.print("Waiting for GPS");
}

void loop() {
  while (ss.available()) {
    gps.encode(ss.read());

    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      int sats = gps.satellites.value();

      Serial.print("Lat: "); Serial.println(lat, 6);
      Serial.print("Lng: "); Serial.println(lng, 6);
      Serial.print("Sats: "); Serial.println(sats);

      lcd.setCursor(0, 0);
      lcd.print("N:");
      lcd.print(abs(lat), 2);
      lcd.print((lat < 0) ? "S" : "N");

      lcd.setCursor(8, 0);
      lcd.print("E:");
      lcd.print(abs(lng), 2);
      lcd.print((lng < 0) ? "W" : "E");

      lcd.setCursor(0, 1);
      lcd.print("Sats:");
      lcd.print("   ");
      lcd.setCursor(6, 1);
      lcd.print(sats);
    }
    else{
      Serial.println("Connecting\n");
    }
  }
  // Serial.println("main loop");
}
