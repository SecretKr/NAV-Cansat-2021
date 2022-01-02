#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_BMP280.h>
MPU9250_DMP imu;
float aX=0,aY=0,aZ=0,gX=0,gY=0,gZ=0;
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
  Serial.println("MPU_Calibrate...\n");
  delay(1000);
  for(int i = 5;i > 0;i--){
    Serial.println(i);
    delay(1000);
  }


  
  for(int i = 0;i < 5;i++){
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      aX += imu.calcAccel(imu.ax);
      aY += imu.calcAccel(imu.ay);
      aZ += imu.calcAccel(imu.az);
      gX += imu.calcGyro(imu.gx);
      gY += imu.calcGyro(imu.gy);
      gZ += imu.calcGyro(imu.gz);
      String mm = String(aX)+","+String(aY)+","+String(aZ)+","+String(gX)+","+String(gY)+","+String(gZ)+"\n";
      Serial.print(mm);
    }
    delay(100);
  }
  aX/=5;
  aY/=5;
  aZ/=5;
  Serial.print("aX : ");
  Serial.println(-aX);
  Serial.print("aY : ");
  Serial.println(-aY);
  Serial.print("aZ : ");
  Serial.println(1-aZ);
  gX/=5;
  gY/=5;
  gZ/=5;
  Serial.print("gX : ");
  Serial.println(-gX);
  Serial.print("gY : ");
  Serial.println(-gY);
  Serial.print("gZ : ");
  Serial.println(-gZ);
}

void loop()
{
  /*if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      aX = imu.calcAccel(imu.ax);
      aY = imu.calcAccel(imu.ay);
      aZ = imu.calcAccel(imu.az);
      String mm = String(aX)+","+String(aY)+","+String(aZ)+"\n";
      Serial.print(mm);
    }
    delay(100);*/
}
