#include <UH.h>
UH uh;  // create uh object to control a uh
int PRValues[PR_CH_NUM];//

void setup(){
  // initialize serial communication:
  Serial.begin(115200);
  // initialize PhotoReflectors
  uh.initPR();
}

void loop(){
  uh.readPR(PRValues);
    for(int i = 0; i < PR_CH_NUM; i ++){
        Serial.print("CH");
        Serial.print(i);
        Serial.print(":");
        Serial.print(PRValues[i]);
        Serial.print(", ");
      }
    Serial.println("");
    delay(1000);
}

