#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
File myFile;
#define chipSelect 4
MPU9250_DMP imu;
Servo pin;
float aX,aY,aZ,gX,gY,gZ,vec;
Adafruit_BMP280 bmp;
bool bmpflag=1;

/***/
float zg = 0.2;
const int c = 140,o = 40;
float xcal = 0.07;
float ycal = 0.20;
float zcal = 0.11;

float gxcal = -4.68;
float gycal = -4.05;
float gzcal = 1.56;


void setup()
{
  Serial.begin(115200);
  pinMode(SS, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
  Serial.println("SD failed!");
  }
  if (!bmp.begin()) {
    Serial.println(F("BMP280 failed"));
    bmpflag = 0;
  }
  else{
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  }
  imu.begin();
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(20);
  imu.setSampleRate(40);
  imu.setCompassSampleRate(10);
  pin.attach(2);
  pin.write(c);
  pinMode(3,INPUT);
  myFile = SD.open("log.csv", FILE_WRITE);
  if(myFile){
    myFile.println("\nMillis,T,AX,AY,AZ,Vec,ReMPU,RePAL,Temp,Press,Alt");
    myFile.close();
  }
}

int launch = 0;
unsigned long t = 0,start;
int rempu = 0;
int repal = 0;
void loop() {
  if (imu.dataReady())
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    aX = imu.calcAccel(imu.ax)+xcal;
    aY = imu.calcAccel(imu.ay)+ycal;
    aZ = imu.calcAccel(imu.az)+zcal;
    gX = imu.calcGyro(imu.gx)+gxcal;
    gY = imu.calcGyro(imu.gy)+gycal;
    gZ = imu.calcGyro(imu.gz)+gzcal;
    vec = sqrt( pow(aX,2) + pow(aY,2) + pow(aZ,2) );
  }
  if(launch < 2 && vec >= 2) launch++;

//mpu
  if(vec <= zg && launch==2) rempu++;
  else rempu = 0;

//palord 
  if(!digitalRead(3) && launch==2 && t>=80) repal++;
  else repal = 0;

  if(t==80 && rempu>3){
    rempu-=20;
  }
  
  
  String mm = String(millis())+","+String(t)+","+String(aX)+","+String(aY)+","+String(aZ)+","+String(vec)+","+String(rempu)+","+String(repal);
  if(bmpflag) mm += ","+String(bmp.readTemperature())+","+String(bmp.readPressure()/100)+","+String(bmp.readAltitude(1019.66));
  mm += ","+String(gX)+","+String(gY)+","+String(gZ);
  myFile = SD.open("log.csv", FILE_WRITE);
  if(myFile){
    myFile.println(mm);
    myFile.close();
  }
  else Serial.println("\nerror write sd card");
  Serial.println(mm);

  
  if((rempu >= 3  && t>=80) || repal >= 4 || t>140){
    pin.write(o);
  }
  //else pin.write(c);
  delay(100-millis()%100);
  if(launch == 2) t++;
}
