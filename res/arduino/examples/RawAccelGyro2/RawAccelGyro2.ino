#include <UH.h>
UH uh;  // create uh object to control a uh

int accelRaw[3];
int gyroRaw[3];

void setup(){
  // initialize serial communication:
  Serial.begin(115200);
  // initialize AcceleGyro
  uh.setupAcceleGyro();
}

void loop(){
  // get the sensor values
  uh.readRawAccelValues(accelRaw);
  uh.readRawGyroValues(gyroRaw);

  Serial.print("accel x: ");
  Serial.print(accelRaw[0]); 
  Serial.print(", accel y: ");
  Serial.print(accelRaw[1]); 
  Serial.print(", accel z: ");
  Serial.println(accelRaw[2]); 

  Serial.print("gyro x: ");
  Serial.print(gyroRaw[0]); 
  Serial.print(", gyro y: ");
  Serial.print(gyroRaw[1]); 
  Serial.print(",gyro z: ");
  Serial.println(gyroRaw[2]); 
  Serial.println("");

  delay(1000);
}

