#include <UH.h>
UH uh;  // create uh object to control a uh

void setup(){
  // initialize serial communication:
  Serial.begin(115200);
  // initialize PhotoReflectors
  uh.initPR();
}

void loop(){
  Serial.println(uh.readPR(0));
  delay(1000);
}

