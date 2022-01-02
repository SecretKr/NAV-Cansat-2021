#include <SoftwareSerial.h>
SoftwareSerial HC12(3, 2); // HC-12 TX Pin, HC-12 RX Pin
const int tp = 13;
int ep = 12;
void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               
}
void loop() {
  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());
  }
}


/*
void loop()
{
  Serial.println(Distance());
  delay(1000);
}
int distance = 999;


long Distance() {
    long d = 0;
    int duration = 0;
    digitalWrite(tp, LOW);
    delayMicroseconds(2);
    digitalWrite(tp, HIGH);
    delayMicroseconds(10);
    digitalWrite(tp, LOW);
    delayMicroseconds(2);
    duration = pulseIn(ep, HIGH, 150000L);
    d = MicrosecondsToCentimeter(duration);
    delay(25);
    return d;
}
long MicrosecondsToCentimeter(long duration) {
    long d = (duration * 100) / 5882;
    //d = (d == 0)?999:d;
    return d;
}*/
