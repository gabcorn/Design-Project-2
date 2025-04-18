#include <SoftwareSerial.h>

SoftwareSerial rxtxSerial(0,1);

void setup() 
{
  rxtxSerial.begin(9600);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  if (Serial.available())
  {
    String data = Serial.readStringUntil('\n');
    if (data.length() > 0)
    {
      Serial.println("Data Being Transmitted");
      rxtxSerial.println(data);
      Serial.println("Data sent");
      Serial.println(data);
      digitalWrite(LED_BUILTIN, 1);  
    }
  }
  delay(100);
}
