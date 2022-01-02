#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
SoftwareSerial HC12(2, 3); //RX, TX
SoftwareSerial GPS(7, 8); //RX, TX
TinyGPSPlus myGPS;
File myFile; // สร้างออฟเจก File สำหรับจัดการข้อมูล
const int chipSelect = 4;
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float latitude = 0, longitude = 0, speed;
unsigned long delayTime;
float z = 0;
void setup()
{
  //mySerial.begin(9600);
Serial.begin(115200);
Serial.println("start");
GPS.begin(9600);
  HC12.begin(9600);
  HC12.print("AT + B9600");
  HC12.print("AT + C005"); 

Serial.print("1");
Wire.begin();                      // Initialize comunication
Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
Wire.write(0x6B);                  // Talk to the register 6B
Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
Wire.endTransmission(true);
Serial.print("2");
bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  }
Serial.print("Initializing SD card...");
pinMode(SS, OUTPUT);
pinMode(13,OUTPUT);
z = bme.readAltitude(SEALEVELPRESSURE_HPA);
delay(100);
if (!SD.begin(4)) {
Serial.println("initialization failed!");
digitalWrite(13,LOW);
return;
}
Serial.println( "initialization done.");
digitalWrite(13,HIGH);
}

void loop()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ ( -0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  

  getPosition();
  
  String mm;
  mm = "\"" + String(bme.readHumidity()) + "\",\"" + String(bme.readTemperature())+"\",\""+String(bme.readPressure() / 100.0F)+"\",\""+String(bme.readAltitude(SEALEVELPRESSURE_HPA)-z)+"\"";
  mm += ",\"" + String(pitch) + "\",\"" + String(yaw)+"\",\""+String(roll)+"\",\""+String(accAngleX)+"\"";
  mm += ",\"" + String(accAngleY) + "\",\"" + String(latitude)+"\",\""+String(longitude)+/*"\",\""+String(speed)+*/"\"";
  //Serial.println(mm);

  S();
  delay(2000);
  Serial.println(mm);
 
  myFile = SD.open("test.csv", FILE_WRITE);
  if (myFile) {
  myFile.println(mm); // สั่งให้เขียนข้อมูล
  myFile.close(); // ปิดไฟล์
  }
}

void S(){
  int la1 = latitude*100;
  int la2 = ((latitude*100)-la1)*10000;
  int lo1 = longitude*100;
  int lo2 = ((longitude*100)-lo1)*10000;
  int a = int(bme.readHumidity());
  int b = int(bme.readTemperature());
  int c = int(bme.readPressure() / 100.0F);
  int d = int(bme.readAltitude(SEALEVELPRESSURE_HPA)-z);
  HC12.println(12345);
  delay(20);
  HC12.println(a);
  delay(20);
  HC12.println(b);
  delay(20);
  HC12.println(c);
  delay(20);
  HC12.println(d);
  delay(20);
  HC12.println(la1);
  delay(20);
  HC12.println(la2);
  delay(20);
  HC12.println(lo1);
  delay(20);
  HC12.println(lo2);
  delay(20);
  HC12.println(accAngleX*100);
  delay(20);
  HC12.println(accAngleY*100);
}

bool getPosition() {
  while (GPS.available() > 0)
    if (myGPS.encode(GPS.read())){
      if (myGPS.location.isValid()) {
        latitude = myGPS.location.lat();
        longitude = myGPS.location.lng();
      }
    }
}

void serialFlush() {
  while(Serial.available()) Serial.read();
}
