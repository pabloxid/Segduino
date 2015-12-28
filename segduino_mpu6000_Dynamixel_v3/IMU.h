
/*
   IMU helper class by Alvaro Cassinelli & Pablo Gindel
*/

#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "MPU6000.h"
#include <SPI.h>
#include "types.h"


class IMU {

  public:
    // Constructor:
    IMU();

    void init();
  
    void reset() {lastUpdate=micros(); estimatedInclination=0; gyroInclination=0;}

    void getAcc(double& _ax, double& _ay, double& _az) {
      _ax = Acc[0];
      _ay = Acc[1];
      _az = Acc[2];
    }
    void getGyro(double& _gx, double& _gy, double& _gz) {
      _gx = Gyro[0];
      _gy = Gyro[1];
      _gz = Gyro[1];
    }
    
    float getEstimatedInclination() {
      return (estimatedInclination); // output angle in degrees
    }
    
    float getAccInclination() {
      return (accInclination); // output angle in degrees
    }
    
    float getGyroInclination() {
      return (gyroInclination); // output angle in degrees
    }
    
    void getAcc(double _A[]) {_A[0]=Acc[0]; _A[1]=Acc[1]; _A[2]=Acc[2];}
    void getGyro(double _G[]) {_G[0]=Gyro[0]; _G[1]=Gyro[1]; _G[2]=Gyro[2];}

    void update(); 

  private:

     unsigned long lastUpdate;
    
     //NOTE: there will be only ONE IMU object. We need to set the readIMU function as a callback, so the method should be static.. but
     // then the data should ALSO be static:
 
     float Acc[3], Gyro[3];    // this is the acceleration (in m/s^2) and gyro data (in rad/sec)
     float gyroInclination, accInclination, estimatedInclination;
     
     RecursiveFilter hpf, lpf;  
    
     MPU6000 mpu6000; 
     
};

#endif;

