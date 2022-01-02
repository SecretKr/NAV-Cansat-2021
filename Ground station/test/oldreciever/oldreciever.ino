
#include <SoftwareSerial.h>

SoftwareSerial HC12(2, 3); // RX, TX


void setup() {
  HC12.begin(9600);
  HC12.print("AT + B9600");
  HC12.print("AT + C008"); 
  Serial.begin(9600);
  //Serial.print("start");
}
String message;
int in,a,b,c,d,e,f,g,h,i,j;
bool F;
void loop() {
  if(HC12.available()){
    in = HC12.parseInt();
    if(in == 12345){
      Serial.println(in);
      F = 1;
      while(F) if(HC12.available()) {a = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {b = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {c = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {d = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {e = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {f = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {g = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {h = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {i = HC12.parseInt(); F = 0;}
      F = 1;
      while(F) if(HC12.available()) {j = HC12.parseInt(); F = 0;}
      //Serial.print("Humidity = ");
      Serial.println(a);
      //Serial.print("Temperature = ");
      Serial.println(b);
      //Serial.print("Pressure = ");
      Serial.println(c);
      //Serial.print("Altitude = ");
      Serial.println(d);
      //Serial.print("Lattitude = ");
      Serial.print(e/100);
      Serial.print(".");
      if(e%100 < 10) Serial.print("0");
      Serial.print(e%100);
      Serial.println(f);
      //Serial.print("Longitude = ");
      Serial.print(g/100);
      Serial.print(".");
      if(g%100 < 10) Serial.print("0");
      Serial.print(g%100);
      Serial.println(h);
      
      Serial.print(i/100);
      Serial.print(".");
      Serial.println(abs(i)%100);
      Serial.print(j/100);
      Serial.print(".");
      Serial.println(abs(j)%100);
    }
  }
}
