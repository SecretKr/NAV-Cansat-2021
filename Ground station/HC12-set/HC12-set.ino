#include <SoftwareSerial.h>
SoftwareSerial HC12(3, 2); // HC-12 TX Pin, HC-12 RX Pin
const int tp = 13;
int ep = 12;
void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               
}
void loop() {
  HC12.println("AT+C069");
  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());
  }
  delay(1000);
}
