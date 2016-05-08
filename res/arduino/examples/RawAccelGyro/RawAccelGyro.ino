#include <UH.h>
UH uh;
int accelRaw[3];
int gyroRaw[3];

void setup(){
  Serial.begin(115200);
  uh.setupAcceleGyro();
}

void loop(){
  uh.readRawAccelValues(accelRaw);
  uh.readRawGyroValues(gyroRaw);
  for(int i=0;i<3;i++){
    Serial.print(accelRaw[i]); 
    Serial.print(gyroRaw[i]); 
  }
  delay(1000);
}

