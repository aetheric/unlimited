//Test for Vibration Motor
void test1() {
  Serial.println("Entering TEST MODE 1");
  while (tCount < 2 ) {
     uh.moveVibrationMotor(300);
     delay(100);
      
    //moveVibrationMotor(100);
    //checkSerialSignal();
    serialEvent();
    if(1<tCount){
      break;
    }
  }
  
}

//Test for Muscle Motion Sensor
void test2() {
  Serial.println("Entering TEST MODE 2");
  while (tCount < 3) {
    uh.checkPR_test();
    delay(1000);
    serialEvent();
    //checkSerialSignal();
    if(2<tCount){
      break;
    }
  } 
}

//Test for accelerometer and gyro
void test3() {
  Serial.println("Entering TEST MODE 3");
  while (tCount < 4) {
    //readAcceleGyro();
    uh.checkAccelGyro_XYZ_Kalman_test();
    delay(1000);
    serialEvent();
    //checkSerialSignal();
    if(3<tCount){
      break;
    }
  }
}

//Test for EMS
void test4() {
  Serial.println("Entering TEST MODE 4");
  while (tCount < 5) {
    for (int i = 0; i < 8; i++) {
      stimulation(i);
      delay(500);
      serialEvent();
    }
    serialEvent();
    if(4<tCount){
      break;
    }
  }
}

void stimulation(int EMSCh) {
  if (EMSCh == 0) {
  // EMSTimeCount = stimuTimeCount;
         uh.setStimulationChannel(0);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
  } else if (EMSCh == 1) {
   // EMSTimeCount =stimuTimeCount;
        uh.setStimulationChannel(1);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
  } else if (EMSCh == 2) {
   // EMSTimeCount =stimuTimeCount;
         uh.setStimulationChannel(2);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
  } else if (EMSCh == 3) {
  //  EMSTimeCount =stimuTimeCount;
         uh.setStimulationChannel(3);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
  } else if (EMSCh == 4) {
   // EMSTimeCount =stimuTimeCount;
         uh.setStimulationChannel(4);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);   
  } else if (EMSCh == 5) {
   // EMSTimeCount =stimuTimeCount;
         uh.setStimulationChannel(5);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);     
   
  } else if (EMSCh == 6) {
   // EMSTimeCount =stimuTimeCount;
         uh.currentEMSChannel = 6;
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel); 
           
  } else if (EMSCh == 7) {
  //  EMSTimeCount =stimuTimeCount;
        uh.setStimulationChannel(6);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         Serial.print("Vol:"); Serial.print(currentVol);
         Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
  }
  for(int i=0;i<400;i++){
    //stimulateD();
    uh.updateEMS();
    
  }
  delay(3000);
}
