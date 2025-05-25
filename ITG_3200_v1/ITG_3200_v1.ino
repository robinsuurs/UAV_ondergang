#include <Wire.h>
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;
char DLPF_CFG_0 = (1<<0);
char DLPF_CFG_1 = (1<<1);
char DLPF_CFG_2 = (1<<2);
char DLPF_FS_SEL_0 = (1<<3);
char DLPF_FS_SEL_1 = (1<<4);
char itgAddress = 0x69;

float scaleFactor = 14.375;
float biasX = 0, biasY = 0, biasZ = 0;
float angleX = 0, angleY = 0, angleZ = 0;
unsigned long lastTime = 0;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  char id=0;
  id = itgRead(itgAddress, 0x00);
  Serial.print("ID: ");
  Serial.println(id, HEX);
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  itgWrite(itgAddress, SMPLRT_DIV, 9);
  // Kalibratie (optioneel)
  const int calSamples = 1000;
  delay(100);
  long sumX=0, sumY=0, sumZ=0;
  for(int i=0; i<calSamples; i++) {
    int x = readX();
    int y = readY();
    int z = readZ();
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }
  biasX = sumX / float(calSamples);
  biasY = sumY / float(calSamples);
  biasZ = sumZ / float(calSamples);
  Serial.println("Calibration done!");
}

float applyDeadzone(float value, float threshold) {
  return (abs(value) < threshold) ? 0.0 : value;
}

void loop()
{
  int rawX = readX();
  int rawY = readY();
  int rawZ = readZ();

  float x_dps = applyDeadzone((rawX - biasX) / scaleFactor, 0.2);
  float y_dps = applyDeadzone((rawY - biasY) / scaleFactor, 0.2);
  float z_dps = applyDeadzone((rawZ - biasZ) / scaleFactor, 0.2);


  Serial.print("X: "); Serial.print(x_dps); 
  Serial.print(" °/s, Y: "); Serial.print(y_dps); 
  Serial.print(" °/s, Z: "); Serial.println(z_dps);
  delay(100);
}
void itgWrite(char address, char registerAddress, char data)
{
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}
void integrateGyro(float x_dps, float y_dps, float z_dps, float dt, 
                   float &angleX, float &angleY, float &angleZ) {
  angleX += x_dps * dt;
  angleY += y_dps * dt;
  angleZ += z_dps * dt;
}

unsigned char itgRead(char address, char registerAddress)
{
  unsigned char data=0;

  Wire.beginTransmission(address);

  Wire.write(registerAddress);

  Wire.endTransmission();

  Wire.requestFrom(address, 1);

  if(Wire.available()){
    data = Wire.read();
  }

  return data;
}

int readX(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_XOUT_L);

  return data;
}

int readY(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_YOUT_L);

  return data;
}

int readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);

  return data;
}
