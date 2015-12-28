
#include "IMU.h"
#include "settings.h"

// Constructor:
IMU::IMU() : hpf (HPF, IMU_FILTER_CUTOFF), lpf (LPF, IMU_FILTER_CUTOFF)
{
  // Calibrate sensors:   
}

void IMU::init() {
  // MPU6000
  Serial.println("Doing IMU startup...");
  mpu6000.init ();
  mpu6000.set_gyro_offsets_scaled (-0.0340, -0.0070,  0.0479);
}

void IMU::update()
{
  float dt=micros()-lastUpdate;
  
  mpu6000.update();
  mpu6000.get_gyros (Gyro);
  mpu6000.get_accels (Acc);
  
  lastUpdate=micros();
  
  // The integration of gyro_X gives the gyroInclination:
  gyroInclination += ToDeg(Gyro[1]) * dt / 1000000.0;           // Note the minus sign because of the axis orientation. It in degrees.

  // From the accelerometer, we can obtain the accInclination:
  accInclination = ToDeg(atan2(Acc[2], Acc[0]));
  
  // Finally, we can use a complimentary filter to get the estimatedInclination:
  estimatedInclination = hpf.update (gyroInclination) + lpf.update(accInclination);

}







