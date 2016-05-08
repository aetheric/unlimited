/*
  UnlimitedHand ver 1.4.12
  This code for the EMS MUX circuit with UnlimitedHand(UH27)
  Written by Emi Tamaki, 2015.07.12
  fixed by Emi Tamaki, 2015.09.20
  fixed by Ken Iwasaki, 2016.03.09 added I2C protocol
  added the virtual multi thread by Emi Tamaki, 2016.03.15
  deleted the virtual multi thread by Emi Tamaki, 2016.04.13
  modified the EMS functions by Emi Tamaki, 2016.04.18
*/

#include <Kalman.h>
#include <UH.h>
UH uh;

int inByte = 0;        //Objects for the Serial Connection to the Computer
int tCount = 0;        //Object for the Test Mode Connt 
int currentVol;        //Voltage for the EMS(Electric Mucsle Stimulation)
///////////////////////////////////////////////////////////////////////////////
// I2C for the Acceleration and the  Gyro                                    //
///////////////////////////////////////////////////////////////////////////////
int accelRaw[3];
int gyroRaw[3];
int tempRaw;
double temp;


///////////////////////////////////////////////////////////////////////////////
//   SET UP
///////////////////////////////////////////////////////////////////////////////
void setup() { 
  delay(500);                     // time to start serial monitor
  Serial.begin(115200);           // Serial.println("UH Start");
  
  //according to docs this holds until serial is open, this does not appear to work
  while (!Serial);  

  uh.setupVibrationMotor();        // set up the Vibration Motor in UnlimitedHand 
  uh.setupAcceleGyro();            // start the I2C connection for the Acceleration and the Gyro
  
  uh.readRawAccelValues(accelRaw); // read the raw accelaration values for the Kalman setup
  uh.readRawGyroValues(gyroRaw);   // read the raw Gyro values for the Kalman setup
  uh.initAccelGyroKalman(micros(), accelRaw); // setup the Kalman
  
  uh.initEMS();                    // setup the EMS(Electric Muscle Stimulation) to output the haptic feeling 
  currentVol = 11;                 // define the voltage of the EMS
  
  uh.initPR();                     // setup the Photo-reflectors to detect hand movements.
  
  printHelp();
}


///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////
void loop() {
  uh.updateEMS();
}



void printHelp(){
  Serial.println("[HOW TO USE THIS ARDUINO CODE ON THE SERIAL MONITOR]");
  Serial.println("a: display the raw values of the acceleration and Gyro sensors");
  Serial.println("A: display the forearm angles");
  Serial.println("b: move the vibration motor for 0.3 sec");
  Serial.println("B: move the vibration motor for 1 sec");
  Serial.println("c: check the Photo-reflectors values");
  Serial.println("d: turn on the VibrationMotor");
  Serial.println("e: turn off the VibrationMotor");
  Serial.println(" ");
  Serial.println("0~7:give the EMS(Electric Muscle Stimulation)");
  Serial.println("h: increase the voltage of the EMS.,  l: decrease the voltage of the EMS");
  Serial.println("m: increase the stimulation time of the EMS.,  n: decrease the stimulation time of the EMS");
  Serial.println(" ");
  Serial.println("T: change the TEST mode");
  Serial.println("------------------------------------------------------------");
  Serial.println("Please input any charactor:");

}



