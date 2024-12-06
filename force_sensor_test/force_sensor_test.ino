// https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3

#include <Adafruit_TinyUSB.h>

void setup() {
  Serial.begin(460800);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  int p1=analogRead(A2);
  int p2=analogRead(A4);
  int diff=p2-p1;
  if (diff<10){ digitalWrite(LED_BUILTIN, HIGH);}
  else digitalWrite(LED_BUILTIN, LOW);
  Serial.println(diff);
}