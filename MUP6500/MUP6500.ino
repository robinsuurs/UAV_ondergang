#include <MPU6500_WE.h>
#include <Wire.h>
#include <math.h>
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
xyzFloat anglegyr = {0.0,0.0,0.0};
unsigned long startTijd;
unsigned long vorigeTijd = millis();
unsigned long dt = 0.0;
String input;
bool metenActief = true;

void setup() {
  Serial.begin(115200);
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
  myMPU6500.setGyrDLPF(MPU6500_DLPF_5);
  myMPU6500.setSampleRateDivider(3);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_2000);
  delay(200);
  startTijd = millis();
}

void loop() {
  instellen();
  if (metenActief){
    meten();}
}

xyzFloat filter(xyzFloat gyr){
  xyzFloat adjustedgyr;
  float threshold = 0.6;
  adjustedgyr.x = (abs(gyr.x) < threshold)? 0.0 : gyr.x;
  adjustedgyr.y = (abs(gyr.y) < threshold)? 0.0 : gyr.y;
  adjustedgyr.z = (abs(gyr.z) < threshold)? 0.0 : gyr.z;
  return adjustedgyr;
}
xyzFloat integrate(xyzFloat gyr){
  unsigned long huidigeTijd = millis();
  dt = (huidigeTijd - vorigeTijd) / 1000;
  vorigeTijd = huidigeTijd;
  anglegyr.x += round(gyr.x * dt);
  anglegyr.y += round(gyr.y * dt);
  anglegyr.z += round(gyr.z * dt);
  anglegyr.x = fmod(anglegyr.x + 180.0, 360.0);
  // Reset naar 0 als hoek buiten bereik -360° tot +360°
  if (anglegyr.x > 360.0 || anglegyr.x < -360.0) anglegyr.x = 0.0;
  if (anglegyr.y > 360.0 || anglegyr.y < -360.0) anglegyr.y = 0.0;
  if (anglegyr.z > 360.0 || anglegyr.z < -360.0) anglegyr.z = 0.0;
  return anglegyr;
}
void instellen(void){
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input == 'r' || input == 'R'){
      Serial.println("Hij is gereset");
      anglegyr = {0.0, 0.0, 0.0};
      dt = 0.0;
  } if (input == 's' || input == 'S'){
    metenActief = !metenActief;   
    if (metenActief){
    Serial.println("Hij gaat meten");      
    }if (!metenActief){
    Serial.println("Meten is gestopt");
    }}
   if (input =='c' || input == 'C'){
    Serial.println("Hij gaat opnieuw kalibreren, houd hem stil");
    myMPU6500.autoOffsets();
    Serial.println("Hij is klaar, begint met meten");
  }
  }}
void meten(void){
  xyzFloat rawgyr = myMPU6500.getGyrValues();
      xyzFloat gyr = filter(rawgyr);
      xyzFloat angle = integrate(rawgyr);
      float temp = myMPU6500.getTemperature();
      Serial.println("Gyroscope data in and degrees: ");
      Serial.print(angle.x);
      Serial.print("   ");
      Serial.print(angle.y);
      Serial.print("   ");
      Serial.println(angle.z);
      Serial.print("Temperature in °C: ");
      Serial.println(temp);
      Serial.println("********************************************");
      delay(1000);
}