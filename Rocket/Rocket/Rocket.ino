#include <SparkFunMPU9250-DMP.h>
#include <Servo.h>
Servo pin;
#define SerialPort Serial
const int c = 140,o = 1;

MPU9250_DMP imu;

void setup()
{
  SerialPort.begin(115200);
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(5);
  imu.setSampleRate(10);
  imu.setCompassSampleRate(10);
  pin.attach(2);
  pin.write(c);
  pinMode(13,OUTPUT);
  pinMode(3,INPUT_PULLUP);
}

float aX,aY,aZ;
float zg = 0.2;

void loop()
{
  bool re = 0;
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    aX = imu.calcAccel(imu.ax);
    aY = imu.calcAccel(imu.ay);
    aZ = imu.calcAccel(imu.az);
    if((aX >= -zg && aX <= zg) && (aY >= -zg && aY <= zg) && (aZ >= -zg && aZ <= zg)){
      re = 1;
    }
  }
  if(!digitalRead(3)){
    re = 1;
    for(int i = 0;i < 5;i++){
      if(digitalRead(3)) re = 0;
      delay(100);
    }
  }


  
  if(re){
    pin.write(o);
    digitalWrite(13,1);
    while(!digitalRead(3)) delay(100);
    delay(5000);
    digitalWrite(13,0);
    pin.write(c);
  }
  else{
    digitalWrite(13,1);
    pin.write(c);
    digitalWrite(13,0);
  }
  delay(100);
}
