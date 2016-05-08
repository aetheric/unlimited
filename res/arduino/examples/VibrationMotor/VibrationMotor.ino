#include <UH.h>
UH uh;  // create uh object to control a uh


void setup(){
  // initialize serial communication:
  Serial.begin(115200);
  // initialize vibrationMotor
  uh.setupVibrationMotor();
}

void loop(){
  uh.onVibrationMotor();   // turn the vibrationMotor on
  delay(1000);              // wait for a second
  uh.offVibrationMotor();    // turn the vibrationMotor off
  delay(1000);              // wait for a second
}

