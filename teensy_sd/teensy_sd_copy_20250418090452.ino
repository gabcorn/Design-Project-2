#include <SD.h>
#include <SoftwareSerial.h>
File file;
SoftwareSerial dStream(21, 20); // RX, TX
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  
  dStream.begin(9600);
  if (!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD card initialization failed");
  }
  
  file = SD.open("test.txt");

  Serial.println("AHHHH I made the thing AHHHH");

}

void loop() {

  file.println("Test line");
  Serial.println("Builtin serial");
  dStream.println("Extern Serial");

}
