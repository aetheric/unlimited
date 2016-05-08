#include <UH.h>
UH uh;  // create uh object to control a uh

int accelRaw[3];
int gyroRaw[3];
int angleXYZ[3];

void setup(){
  // initialize serial communication:
  Serial.begin(115200);
  // initialize AcceleGyro
  uh.setupAcceleGyro();
  // get the sensor values
  uh.readRawAccelValues(accelRaw);
  // initialize Kalman  
  uh.initAccelGyroKalman(micros(), accelRaw);
}

void loop(){
  // get the sensor values
  uh.readRawAccelValues(accelRaw);
  uh.readRawGyroValues(gyroRaw);
  uh.readAccelGyro_XYZ_Kalman(accelRaw,gyroRaw,angleXYZ);

  Serial.print("angle x: ");
  Serial.print(angleXYZ[0]); 
  Serial.print(", angle y: ");
  Serial.print(angleXYZ[1]); 
  Serial.print(", angle z: ");
  Serial.println(angleXYZ[2]); 

  delay(1000);
}

