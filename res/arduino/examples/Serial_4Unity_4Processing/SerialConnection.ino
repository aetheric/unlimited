
void serialEvent() {
  if(Serial.available()>0){
    inByte = Serial.read();
    switch (inByte) {
      case 48:                    //EMS 0
         uh.setStimulationChannel(0);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);  
         break;
      case 49:                    //EMS 1
         uh.setStimulationChannel(1);
         uh.setStimulationTime();
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);          
         break;
      case 50:                    //EMS 2
         uh.setStimulationTime();
         uh.setStimulationChannel(2);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);         
         break;
      case 51:                    //EMS 3
         uh.setStimulationTime();
         uh.setStimulationChannel(3);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);            
         break;
      case 52:                    //EMS 4
         uh.setStimulationTime();
         uh.setStimulationChannel(4);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);            
         break;
      case 53:                    //EMS 5
         uh.setStimulationTime();
         uh.setStimulationChannel(5);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);            
         break;
      case 54:                     //EMS 6
         uh.setStimulationTime();
         uh.setStimulationChannel(6);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);            
         break;
      case 55:                    //EMS 7
         uh.setStimulationTime();
         uh.setStimulationChannel(7);
         uh.setStimulationVoltage(currentVol);
         for(int i=0;i<50;i++){uh.keepVoltage(currentVol);}
         //Serial.print("Vol:"); Serial.print(currentVol);
         //Serial.print(", EMS num: "); Serial.println(uh.currentEMSChannel);            
         break;
      case 97:                                    //a(Acceleration and Gyro)
          uh.readRawAccelValues(accelRaw);
          uh.readRawGyroValues(gyroRaw);
          temp = uh.readTemperature();
          Serial.print(accelRaw[0], DEC);//AcX = 
          Serial.print(", "); Serial.print(accelRaw[1], DEC);// | AcY =
          Serial.print(", "); Serial.print(accelRaw[2], DEC);// | AcZ = 
          Serial.print(", "); Serial.print(temp, 3);  //temperature
          Serial.print(", "); Serial.print(gyroRaw[0], DEC);
          Serial.print(", "); Serial.print(gyroRaw[1], DEC);
          Serial.print(", "); Serial.println(gyroRaw[2], DEC);
          delay(1);
         break;
      case 65:                                    //A(Acceleration and Gyro) with kalman
         uh.checkAccelGyro_XYZ_Kalman();
         break;
      case 98:                                    //b(move the vibration motor)
         uh.moveVibrationMotor(300);
         Serial.print("move the Vibration Motor. Time(sec):");
         Serial.println(300);
         break;
      case 66:                                    //B(move the vibration motor)
         uh.moveVibrationMotor(1000);
         Serial.print("move the Vibration Motor. Time(sec):");
         Serial.println(1000);
         break;
      case 99:                                    //c(check the Photo-reflectors)
        uh.checkPR();
        break;
      case 100:                                   //d
        uh.onVibrationMotor();
        break;
      case 101:                                    //e
        uh.offVibrationMotor();
        break;
      case 102:                                    //f
        for(int p=0;p<20;p++){
          uh.checkRawAccelValues();
          uh.checkTemperature();
          uh.checkRawGyroValues();
          Serial.print(" ,PR, ");
          uh.checkPR();
          Serial.println(" ");
        }
        break;
      case 103:                                    //g
        //checkPR();
        break;
      case 104:                                    //ｈ
         if(currentVol < 12){
           currentVol = currentVol + 1;
           Serial.print("Vol:");
           Serial.println(currentVol);
         }else if(12 <= currentVol){
           Serial.println("Vol:12, it is maximum of the EMS Voltage");
         }
         break;
      case 108:                                    //l
         if(0 < currentVol){
           currentVol = currentVol - 1;
           Serial.print("Vol:");
           Serial.println(currentVol);
         }else if(currentVol <= 0){
           Serial.println("Vol:0, it is mimimum of the EMS Voltage");
         }
         break;

      ////stimuTimeCount///////////////////////////////////////   
      case 109:                                    //m
         if(uh.stimuTimeCount<=DEF_STIMU_TIME_COUNT*3){
           uh.stimuTimeCount = uh.stimuTimeCount + 50;
         }
         Serial.print("stimuTimeCount:");
         Serial.println(uh.stimuTimeCount);
         break;
      case 110:                                    //n
         if(DEF_STIMU_TIME_COUNT/4<=uh.stimuTimeCount){
           uh.stimuTimeCount = uh.stimuTimeCount - 50;
         }
         Serial.print("stimuTimeCount:");
         Serial.println(uh.stimuTimeCount);
         break;  
      case 111:          //o
        if(uh.stimuHighWid<250){
          uh.stimuHighWid += 50;
          Serial.print("stimuHighWid:");
           Serial.println(uh.stimuHighWid);
        }
        break;
      case 112:          //p
        if(100<uh.stimuHighWid){
          uh.stimuHighWid -= 50;
          Serial.print("stimuHighWid:");
           Serial.println(uh.stimuHighWid);
        }
        break;
      
        
     case 84:                    //TEST MODE <T>
         tCount++;
         if(tCount==1){
            test1(); // vibration
         }else if(tCount==2){
            test2(); // muscle motion sensor
         }else if(tCount==3){
            test3(); // accelerometer and gyro
         }else if(tCount==4){
            test4(); // EMS
         }else{
            Serial.println("Exit TEST MODE...");
         }         
         break;
    
                 
      //default:
    }
   /*Serial.print("I received: ");
   Serial.println(inByte, DEC);*/
  }
}

