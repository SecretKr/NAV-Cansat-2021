#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
MPU9250_DMP imu;
float aX,aY,aZ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SS, OUTPUT);
  
  Serial.print("Testing SD card...  ");
  if (!SD.begin(chipSelect)) {
  Serial.println("failed!");
  }
  else Serial.println("OK");
  
  imu.begin();
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(20);
  imu.setSampleRate(40);
  imu.setCompassSampleRate(10);

  Serial.print("Testing MPU...      ");
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    aX = imu.calcAccel(imu.ax);
    aY = imu.calcAccel(imu.ay);
    aZ = imu.calcAccel(imu.az);
    float vec = abs(sqrt( pow(aX,2) + pow(aY,2) + pow(aZ,2) ));
    //String mm = String(aX)+","+String(aY)+","+String(aZ)+","+String(vec)+"\n";
    //Serial.println(mm);
    if(vec <= 0.01 && vec >= -0.01) Serial.println("failed!");
    else Serial.println("OK");
  }
  else Serial.println("failed!");
  Serial.println("Finished");
}

void loop() {
  // put your main code here, to run repeatedly:
/*
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    aX = imu.calcAccel(imu.ax)+0.09;
    aY = imu.calcAccel(imu.ay)+0.10;
    aZ = imu.calcAccel(imu.az)+0.12;
    float vec = sqrt( pow(aX,2) + pow(aY,2) + pow(aZ,2) );
    String mm = String(aX)+","+String(aY)+","+String(aZ)+","+String(vec)+"\n";
    Serial.println(mm);
  }
  delay(100);*/
}
