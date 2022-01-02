#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_BMP280.h>
MPU9250_DMP imu;
float aX=0,aY=0,aZ=0;
Adafruit_BMP280 bmp;
bool bmpflag = 1;

void setup()
{
  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    bmpflag = 0;
  }
  else{
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  }

  
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(20);
  imu.setSampleRate(40);
  imu.setCompassSampleRate(10);
}

void loop()
{
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    aX = imu.calcAccel(imu.ax)+0.49;
    aY = imu.calcAccel(imu.ay)-0.49;
    aZ = imu.calcAccel(imu.az)+0.14;
    float vec = sqrt( pow(aX,2) + pow(aY,2) + pow(aZ,2) );
    String mm = String(aX)+","+String(aY)+","+String(aZ);

    if(bmpflag){
      mm += String(bmp.readTemperature())+","+String(bmp.readPressure()/100)+","+String(bmp.readAltitude(1019.66));
    }
    Serial.println(mm);
  }
  delay(100);
}
