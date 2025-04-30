#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
//IMU includes
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

//IMU + gyro
Adafruit_LIS3MDL compass;
Adafruit_LSM6DSOX imu;
sensors_event_t accel, gyro, temp, mag;
// Serial interfaces
#define btSerial  Serial1
#define gpsSerial Serial2
#define dSerial   Serial3

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
  // Serial.begin(9600);      // USB for GUI/debug
  dSerial.begin(9600);    //non usb-serial Serial port
  btSerial.begin(9600);    // Bluetooth output
  gpsSerial.begin(9600);   // GPS input

  lcd.init();
  lcd.backlight();
  lcd.print("Init SD card...");
  delay(1000);
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
  delay(1000);

  if (!imu.begin_I2C()) {
    lcd.clear();
    lcd.print("IMU FAIL 1");
    while (1);
  }

  if (!compass.begin_I2C()) {
    lcd.clear();
    lcd.print("IMU FAIL 2");
    while (1);
  }

  compass.setPerformanceMode(LIS3MDL_HIGHMODE);
  compass.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  compass.setDataRate(LIS3MDL_DATARATE_155_HZ);
  compass.setRange(LIS3MDL_RANGE_4_GAUSS);
  compass.setIntThreshold(500);
  compass.configInterrupt(false, false, true, // enable z axis
                            true, // polarity
                            false, // don't latch
                            true); // enabled!

  //DEBUG: remove old imu
  // // LSM9DS1 IMU over I2C (Wire)
  // if (!lsm.begin()) {
  //   lcd.clear();
  //   lcd.print("IMU FAIL");
  //   while (1);
  // }

  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  lcd.clear();
  lcd.print("Waiting for GPS");
}
String line;
float lat = -1.000000;
float lng = -1.000000;
float alt =  -1;
int sats = 0;

void loop() {
  // IMU read
  imu.getEvent(&accel, &gyro, &temp);
  compass.getEvent(&mag);

  // lsm.read();
  // accx = lsm.accelData.x;
  // accy = lsm.accelData.y;
  // accz = lsm.accelData.z;

  // avx = lsm.gyroData.x;
  // avy = lsm.gyroData.y;
  // avz = lsm.gyroData.z;

  // magx = lsm.magData.x;
  // magy = lsm.magData.y;
  // magz = lsm.magData.z;
  accx = accel.acceleration.x;
  accy = accel.acceleration.y;
  accz = accel.acceleration.z;
  avx  = gyro.gyro.x;
  avy  = gyro.gyro.y;
  avz  = gyro.gyro.z;
  magx = mag.magnetic.x;
  magy = mag.magnetic.y;
  magz = mag.magnetic.z;

  
  // line = "0,0,0,0," + 
  // String(avx, 2) + "," + String(avy, 2) + "," + String(avz, 2) + "," +
  // String(accx, 2) + "," + String(accy, 2) + "," + String(accz, 2) + "," +
  // String(magx, 2) + "," + String(magy, 2) + "," + String(magz, 2);
  if(!gpsSerial.available()){

    line =
          String(lat, 6) + "," + String(lng, 6) + "," + String(alt, 2) + "," + String(sats) + "," +
          String(avx, 2) + "," + String(avy, 2) + "," + String(avz, 2) + "," +
          String(accx, 2) + "," + String(accy, 2) + "," + String(accz, 2) + "," +
          String(magx, 2) + "," + String(magy, 2) + "," + String(magz, 2);

    dSerial.println(line);
    btSerial.println(line);
    Serial.println(line);
    logFile.println(line);
    logFile.flush();

  }

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
      
    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
      alt = gps.altitude.isValid() ? gps.altitude.meters() : 280;
      sats = gps.satellites.value();

      // Combine into a single line
      line =
        String(lat, 6) + "," + String(lng, 6) + "," + String(alt, 2) + "," + String(sats) + "," +
        String(avx, 2) + "," + String(avy, 2) + "," + String(avz, 2) + "," +
        String(accx, 2) + "," + String(accy, 2) + "," + String(accz, 2) + "," +
        String(magx, 2) + "," + String(magy, 2) + "," + String(magz, 2);

      // Output to USB + Bluetooth
      dSerial.println(line);
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
