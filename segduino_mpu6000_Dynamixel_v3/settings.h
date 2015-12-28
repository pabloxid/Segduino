#ifndef __SETTINGS_H__
#define __SETTINGS_H__


// ajustes

#define ANGLE_UPDATE_TIME      5             // ms

#define THETA_CENTER           -3.3          // degrees

const float THETA_MIN = THETA_CENTER - 2.0;
const float THETA_MAX = THETA_CENTER + 2.0;

#define ANGLE_KP               310.0
#define ANGLE_KI               4000.0
#define ANGLE_KD               2.5

#define IMU_FILTER_CUTOFF      0.0006




#endif
