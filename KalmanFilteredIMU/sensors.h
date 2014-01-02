/*
 * sensors.h
 *
 * Created: 09/12/2011 13:22:37
 *  Author: Loïc KAEMMERLEN
 * Based on the pololu libraries for LSM303 and L3G4200D
 */ 

#include "mathstools.h"

#ifndef COMPASS_H_
#define COMPASS_H_


void compass_config(void);
void compass_read_data(vector *a, vector *m);
void compass_calibration (void);
float get_heading( vector *a, vector *m, vector *p);


void gyro_config(void);
void gyro_read_data(vector *g);


#endif /* COMPASS_H_ */