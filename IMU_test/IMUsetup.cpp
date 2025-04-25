#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <SD.h>
#include <SPI.h>

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSOX sox;
#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 10
#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

File logFile;

void setup(void) {
    Serial.begin(9600);
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
    Serial.println("Adafruit LSM6DSOX+LIS3MDL test!");
    
    // Try to initialize!
    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
      Serial.println("Failed to find LIS3MDL chip");
      while (1) { delay(10); }
    }
    Serial.println("LIS3MDL Found!");
  
    lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(false, false, true, // enable z axis
                            true, // polarity
                            false, // don't latch
                            true); // enabled!

  if (!sox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  if (!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD card initialization failed");
  }

  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
  logFile.println("Temperature,Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z");
  logFile.flush();
  }
  else{
    Serial.println("COuldn't open file");
  }
  

  Serial.println("LSM6DSOX Found!"); 
  }
  

  void loop() {
sensors_event_t accel, gyro, temp;
sox.getEvent(&accel, &gyro, &temp);

sensors_event_t event;
lis3mdl.getEvent(&event);

// Print all sensor values on one line, CSV-style
Serial.print(temp.temperature); Serial.print(",");     // Temperature log
Serial.print(accel.acceleration.x); Serial.print(","); // X acceleration log
Serial.print(accel.acceleration.y); Serial.print(","); // y accelerarion log
Serial.print(accel.acceleration.z); Serial.print(","); // z accelerarion log
Serial.print(gyro.gyro.x); Serial.print(",");          // x gyroscope log
Serial.print(gyro.gyro.y); Serial.print(",");          // y gyroscope log
Serial.print(gyro.gyro.z); Serial.print(",");          // z gyroscope log
Serial.print(event.magnetic.x); Serial.print(",");     // x magnetic field log
Serial.print(event.magnetic.y); Serial.print(",");     // y magnetic field log
Serial.println(event.magnetic.z); // End of line       // z magnetic field log
if (logFile) {
      logFile.print(temp.temperature); logFile.print(",");
      logFile.print(accel.acceleration.x); logFile.print(",");
      logFile.print(accel.acceleration.y); logFile.print(",");
      logFile.print(accel.acceleration.z); logFile.print(",");
      logFile.print(gyro.gyro.x); logFile.print(",");
      logFile.print(gyro.gyro.y); logFile.print(",");
      logFile.print(gyro.gyro.z); logFile.print(",");
      logFile.print(event.magnetic.x); logFile.print(",");
      logFile.print(event.magnetic.y); logFile.print(",");
      logFile.println(event.magnetic.z);
      logFile.flush();
    }
delay(100);
}
