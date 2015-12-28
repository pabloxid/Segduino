/*
   Author: Alvaro Cassinelli & Pablo Gindel
   Ver. 01.6.2015
*/

#include "settings.h"

#include "ax12.h"
AX12 motorL, motorR;

#include <SPI.h>
#include "IMU.h"  
IMU imu;

void updateIMU () {
  ledMonitor ();
  imu.update();
} 

#include "types.h"
Timer<> timerIMU (ANGLE_UPDATE_TIME, updateIMU); 

double angleInput, angleOutput, angleSetpoint;

#include <PID_v1.h>
PID anglePid (&angleInput, &angleOutput, &angleSetpoint, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);  


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                             SETUP                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin (115200);            // PC
  Serial1.begin (57600);            // xbee
  
  // 1) initialize IMU:
  imu.init();              // connection, calibration...

  // 2) initialize anglePID:
  angleSetpoint = THETA_CENTER;
  anglePid.SetSampleTime (ANGLE_UPDATE_TIME);
  anglePid.SetOutputLimits (-1023.0, 1023.0);
  anglePid.SetMode (AUTOMATIC);
    
  // 3) initialize motors
  initMotors ();
  
  // misc.
  initLeds ();
  
  // tranca el programa hasta que el coso esté derecho, para evitar que se acumule error
  float theta;
  while (abs (theta-THETA_CENTER)>.1) {
    imu.update();
    theta = imu.getEstimatedInclination(); // in deg, from -180 to 180
    delay (5);
  }
  
  // arranca
  imu.reset();  // reset timer & integration term
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                             LOOP                                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
 
  timerIMU.update();
  angleInput = imu.getEstimatedInclination();       // in deg, from -180 to 180
  anglePid.Compute();
  motorR.endlessTurn (int(-angleOutput));
  motorL.endlessTurn (int(angleOutput));    // the wheel is reversed...
      
  // 5) Check IMU data:
  // printIMU();

  // 6) Check complimentary filter for estimation of the inclination:
  // printEstimatedInclination();

  // 7) Check PID variables:
  // printPID();
  
  leerSerial ();
  
}

