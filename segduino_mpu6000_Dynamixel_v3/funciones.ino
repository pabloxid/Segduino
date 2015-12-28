
// ===============================================================================================================//
//                                                 MISC. FUNCTIONS                                                //
// ===============================================================================================================//

void initMotors () {
  AX12::init (57142);                                 // Dynamixel MX-28 init
  delay (1);
  byte detect[2];                                     // array ID auto-detection
  byte num = AX12::autoDetect (detect, 2);             
  Serial.print ("Motor detection: ");
  Serial.println (num, DEC);
  motorL = AX12(detect[0]);      
  motorR = AX12(detect[1]);      
  AX12* motors [2] = {&motorL, &motorR};
  for (int i=0; i<2; i++) {
    motors[i]->setSRL (RETURN_READ);
    delay(1);
    motors[i]->torqueOn;                                     // habilita el torque
    delay(1);
    motors[i]->setEndlessTurnMode (true);                    // esto resetearía los angle limits
    delay(1);
    motors[i]->writeInfo (MAX_TORQUE, 1023);
    delay(1);
    motors[i]->writeInfo (LIMIT_TEMPERATURE, 70);
    delay(1);
    motors[i]->writeInfo (DOWN_LIMIT_VOLTAGE, 60);
    delay (5);
    motors[i]->writeInfo (UP_LIMIT_VOLTAGE, 160);
    delay (5);
    motors[i]->writeInfo (RETURN_DELAY_TIME, 50);
    delay (10);
  }
}


#define A_LED_PIN        27    // rojo
#define B_LED_PIN        26    // amarillo
#define C_LED_PIN        25    // azul

void initLeds () {
  pinMode (A_LED_PIN, OUTPUT);
  pinMode (B_LED_PIN, OUTPUT);
  pinMode (C_LED_PIN, OUTPUT);
}

void prenderLeds (int color) {
  digitalWrite (A_LED_PIN, !(color&1));
  digitalWrite (B_LED_PIN, !((color>>1)&1));
  digitalWrite (C_LED_PIN, !((color>>2)&1));
}


#define TOLERANCE  0.1

void ledMonitor () {
  float error = angleSetpoint - angleInput;
  if (abs(error) < TOLERANCE) {
    prenderLeds (AMARILLO);
  } else {
    if (error > 0) {
      prenderLeds (AZUL);
    } else {
      prenderLeds (ROJO);
    }
  }
}



// ===============================================================================================================//
//                                                    PRINTERS                                                    //
// ===============================================================================================================//

void printIMU() {
  
  double A[3], G[3];
  Serial.println("Acc(g): ");
  imu.getAcc(A);
  Serial.print(A[0]);
  Serial.print(" , ");
  Serial.print(A[1]);
  Serial.print(" , ");
  Serial.println(A[2]);

  Serial.print("Gyro(deg/s):  ");
  imu.getGyro(G);
  Serial.print(G[0]);
  Serial.print(" , ");
  Serial.print(G[1]);
  Serial.print(" , ");
  Serial.println(G[2]);

}

void printEstimatedInclination() {

  Serial.print("theta (acc/gyro/filter): ");
  Serial.print(imu.getAccInclination());
  Serial.print("   ");
  Serial.print(imu.getGyroInclination());
  Serial.print("   ");
  Serial.println(imu.getEstimatedInclination());
}

void printPID() {

  Serial.print ("SP1: ");
  Serial.print (angleSetpoint);
  Serial.print (" / PV1: ");
  Serial.print (angleInput);
  Serial.print (" / MV1: ");
  Serial.println (angleOutput);

}
