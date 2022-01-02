#include <Servo.h>
Servo s;
void setup() {
  // put your setup code here, to run once:
  s.attach(6);
  Serial.begin(115200);
}

void loop() {
  while(Serial.available()){
    int d = Serial.parseInt();
    s.write(d);
    Serial.read();
  }
}
