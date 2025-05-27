#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
}

void loop() {
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat rawgyr = myMPU6500.getGyrValues();
  xyzFloat gyr = filter(rawgyr);
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);
  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.print("Temperature in °C: ");
  Serial.println(temp);
  Serial.println("********************************************");
  delay(1000);
}

xyzFloat filter(xyzFloat gyr){
  xyzFloat adjustedgyr;
  float threshold = 0.6;
  adjustedgyr.x = (abs(gyr.x) < threshold)? 0.0 : gyr.x;
  adjustedgyr.y = (abs(gyr.y) < threshold)? 0.0 : gyr.y;
  adjustedgyr.z = (abs(gyr.z) < threshold)? 0.0 : gyr.z;
  return adjustedgyr;
}