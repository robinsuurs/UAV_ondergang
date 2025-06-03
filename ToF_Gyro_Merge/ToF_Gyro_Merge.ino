//Libraries
#include <MPU6500_WE.h>   //GYRO
#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>      //ToF

//GYRO
#define MPU6500_ADDR 0x69
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
xyzFloat anglegyr = {0.0,0.0,0.0};
unsigned long startTijd;
unsigned long vorigeTijd = millis();
unsigned long dt = 0.0;
String input;
bool metenActief_Gyro = true;


//ToF
// XSHUT pin definitions
#define XSHUT_VOOR         2
#define XSHUT_LINKS_VOOR   15
#define XSHUT_LINKS_ACHTER 4

// I2C pin definitions (custom)
#define SDA_PIN 21 // groen
#define SCL_PIN 22 // blauw
VL53L0X sensorVoor, sensorLinksVoor, sensorLinksAchter;
bool metenActief_ToF = true;

// Distance array (mm)
float afstanden[3];
int afstand_fix[3];
float angleDeg;

// Moving average buffer size
const int windowSize = 4;

// Buffers and indices for each sensor
float bufferVoor[windowSize], bufferLinksVoor[windowSize], bufferLinksAchter[windowSize];
int bufferIndexVoor = 0, bufferIndexLinksVoor = 0, bufferIndexLinksAchter = 0;

// Moving average calculation for float
float movingAverage_ToF(float newValue, float *buffer, int *index) {
  buffer[*index] = newValue;
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += buffer[i];
  }
  *index = (*index + 1) % windowSize;
  return sum / windowSize;
}
void setup_ToF(){
  // Initialize XSHUT pins
  pinMode(XSHUT_VOOR, OUTPUT);
  pinMode(XSHUT_LINKS_VOOR, OUTPUT);
  pinMode(XSHUT_LINKS_ACHTER, OUTPUT);

  // Disable all sensors
  digitalWrite(XSHUT_VOOR, LOW);
  digitalWrite(XSHUT_LINKS_VOOR, LOW);
  digitalWrite(XSHUT_LINKS_ACHTER, LOW);
  delay(100);

  // Initialize sensor VOOR
  digitalWrite(XSHUT_VOOR, HIGH);
  delay(100);
  sensorVoor.init();
  sensorVoor.setAddress(0x30);
  sensorVoor.startContinuous();

  // Initialize sensor LINKS VOOR
  digitalWrite(XSHUT_LINKS_VOOR, HIGH);
  delay(100);
  sensorLinksVoor.init();
  sensorLinksVoor.setAddress(0x31);
  sensorLinksVoor.startContinuous();

  // Initialize sensor LINKS ACHTER
  digitalWrite(XSHUT_LINKS_ACHTER, HIGH);
  delay(100);
  sensorLinksAchter.init();
  sensorLinksAchter.setAddress(0x32);
  sensorLinksAchter.startContinuous();
}

void setup_Gyro(){
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

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  //GYRO
  setup_Gyro();
  //ToF
  setup_ToF();
}

void loop(){
  instellen();            //SERIALINPUT
  //GYRO
  if (metenActief_Gyro){
    meten_Gyro();} 
  //ToF
  if (metenActief_ToF){
    meten_ToF();}
}


xyzFloat filter_Gyro(xyzFloat gyr){
  xyzFloat adjustedgyr;
  float threshold = 0.6;
  adjustedgyr.x = (abs(gyr.x) < threshold)? 0.0 : gyr.x;
  adjustedgyr.y = (abs(gyr.y) < threshold)? 0.0 : gyr.y;
  adjustedgyr.z = (abs(gyr.z) < threshold)? 0.0 : gyr.z;
  return adjustedgyr;}
xyzFloat integrate_Gyro(xyzFloat gyr){
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
  return anglegyr;}
void instellen(void){
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input == 'r' || input == 'R'){
      Serial.println("Gyro is gereset");
      anglegyr = {0.0, 0.0, 0.0};
      dt = 0.0;
  } if (input == 'G' || input == 'g'){
    metenActief_Gyro = !metenActief_Gyro;   
    if (metenActief_Gyro){
    Serial.println("Hij gaat meten met Gyro");      
    }if (!metenActief_Gyro){
    Serial.println("Meten is gestopt met Gyro");
    }
  } if (input == 'T' || input == 't'){
    metenActief_ToF = !metenActief_ToF;   
    if (metenActief_ToF){
    Serial.println("Hij gaat meten met ToF");      
    }if (!metenActief_ToF){
    Serial.println("Meten is gestopt met ToF");
    }}
   if (input =='c' || input == 'C'){
    Serial.println("Gyro gaat opnieuw kalibreren, houd hem stil");
    myMPU6500.autoOffsets();
    Serial.println("Gyro is klaar, begint met meten");
  }
  }}
void meten_Gyro(void){
  xyzFloat rawgyr = myMPU6500.getGyrValues();
      xyzFloat gyr = filter_Gyro(rawgyr);
      xyzFloat angle = integrate_Gyro(rawgyr);
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
      delay(1000);}

void readout_ToF() {
  // Raw readings (cast to float)
  float ruweVoor        = (float)sensorVoor.readRangeContinuousMillimeters();
  float ruweLinksVoor   = (float)sensorLinksVoor.readRangeContinuousMillimeters();
  float ruweLinksAchter = (float)sensorLinksAchter.readRangeContinuousMillimeters();

  // Moving averages
  afstanden[0] = movingAverage_ToF(ruweVoor, bufferVoor, &bufferIndexVoor);
  afstanden[1] = movingAverage_ToF(ruweLinksVoor, bufferLinksVoor, &bufferIndexLinksVoor);
  afstanden[2] = movingAverage_ToF(ruweLinksAchter, bufferLinksAchter, &bufferIndexLinksAchter);

  // Correction offset
  float correctiefactor = 0.9004388715;
  float correctiefactor_2 = 0.953;
  if (afstanden[0] < 124 || afstanden[1] < 124 || afstanden[2] < 124){
    afstand_fix[0] = afstanden[0] * correctiefactor_2;
    afstand_fix[1] = afstanden[1] * correctiefactor_2;
    afstand_fix[2] = afstanden[2] * correctiefactor_2;}
    else
    {
    afstand_fix[0] = afstanden[0] * correctiefactor;
    afstand_fix[1] = afstanden[1] * correctiefactor;
    afstand_fix[2] = afstanden[2] * correctiefactor;
    }}

void angle_ToF(){
  // Convert mm to cm
  float distanceVoor_cm = afstand_fix[2] / 10.0;   // LinksVoor
  float distanceAchter_cm = afstand_fix[1] / 10.0; // LinksAchter
  float sensorSpacing_cm = 23.5;

  // Calculate angle
  float hoogteVerschil = distanceAchter_cm - distanceVoor_cm;
  float angleRad = atan2(hoogteVerschil, sensorSpacing_cm);
  angleDeg = angleRad * (180.0 / PI);
  }

void meten_ToF(){
  readout_ToF();
  angle_ToF();
  // Print distances with 1 decimal precision
  Serial.print("Voor: ");
  Serial.print(afstand_fix[0], 0);
  Serial.print(" mm, LinksVoor: ");
  Serial.print(afstand_fix[1], 0);
  Serial.print(" mm, LinksAchter: ");
  Serial.print(afstand_fix[2], 0);
  Serial.println(" mm");
  Serial.print(" cm, Hoek: ");
  Serial.print(angleDeg, 0);
  Serial.println(" graden");
  delay(100);
}