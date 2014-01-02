/*
 * KalmanFilter.h
 *
 * Created: 21/12/2011 16:42:42
 *  Author: Loïc Kaemmerlen
 */ 


#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_


#include "mathstools.h"


//////////////////// ACCELEROMETER+COMPASS PART //////////

// Converts given acceleration raw data in g
void accel_g (vector *a);
// Computes angles from scaled and shifted acceleration data
vector accel_angle (vector a);
// Whole function to get angles from accelerometer
vector accel_angle_acquisition(void);
// Whole function to get angles from accelerometer + compass
vector accelcompass_angle_acquisition(void);
// Calculate the accelerometer measurement noise
vector accel_measurement_noise (void);


//////////////////// GYROSCOPE PART //////////////////////

// Converts given gyroscope raw data in degrees per second
void gyro_dps(vector *g);
// Computes the angle moved using the time between 2 measurements, in degrees
vector gyro_angle (vector g);
// Whole function to get angles from gyroscope
vector gyro_angle_acquisition(void);
// Calculate the gyroscope processing noise
vector gyro_measurement_noise (void);




//////////////////// KALMAN PART //////////////////////

void KalmanFilter (Matrix* xk, Matrix xnew, Matrix uk, Matrix* Pk, Matrix R, Matrix Q, Matrix* S, Matrix* K, Matrix I);



#endif /* KALMANFILTER_H_ */