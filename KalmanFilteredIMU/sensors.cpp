/*
 * sensors.cpp
 *
 * Created: 09/12/2011 13:22:37
 *  Author: Loïc KAEMMERLEN
 
 * Based on the pololu libraries for LSM303 and L3G4200D
 */ 

#include <avr/io.h>  
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>	
	
#include "mathstools.h"
#include "sensors.h"
#include "usart.h"
#include "i2ccomm.h"





// Calibration values using compass_calibration. This needs to be done!

vector m_min = {-492, -503, -549};
vector m_max = {579, 494, 322};


void compass_config(void)
{
	
//CPU Configuration - I2C Configuration
	
 	DDRC = 0;								// all inputs
	PORTC = (1 << PORTC4) | (1 << PORTC5);	// enable pull-ups on SDA and SCL, respectively

	//Prescalar value. TWSR=0 => Prescalar =1
	TWSR = 0;								
	
	// For the Atmega328p, TWBR = ((CPUfreq/SCLfreq) -16)/(2.PRESCALARvalue)
	//Default Prescalar =1
	
	TWBR = 12;								// produces an SCL frequency of 400 kHz with a 16 MHz CPU clock speed

//Compass configuration

	//enable accelerometer
	i2c_start(); 
	i2c_write_byte(0x30); // write acc
	i2c_write_byte(0x20); // CTRL_REG1_A
	i2c_write_byte(0x27); // normal power mode, 50 Hz data rate, all axes enabled
	i2c_stop();
	
	// Configures scale
	i2c_start(); 
	i2c_write_byte(0x30); // write acc
	i2c_write_byte(0x23); // CTRL_REG4_A
	//Next value is 0x00 for +-2g, 0x10 for +-4g, 0x30 for +-8g
	i2c_write_byte(0x00); 
	i2c_stop();

	//enable magnetometer
	i2c_start(); 
	i2c_write_byte(0x3C); // write mag
	i2c_write_byte(0x02); // MR_REG_M
	i2c_write_byte(0x00); // continuous conversion mode
	i2c_stop();
}




// Returns a set of acceleration and raw magnetic readings from the compass.
void compass_read_data(vector *a, vector *m)
{
	// read accelerometer values
	i2c_start();
	i2c_write_byte(0x30); // write acc
	i2c_write_byte(0xa8); // OUT_X_L_A, MSB set to enable auto-increment
	i2c_start();		  // repeated start
	i2c_write_byte(0x31); // read acc
	unsigned char axl = i2c_read_byte();
	unsigned char axh = i2c_read_byte();
	unsigned char ayl = i2c_read_byte();
	unsigned char ayh = i2c_read_byte();
	unsigned char azl = i2c_read_byte();
	unsigned char azh = i2c_read_last_byte();
	i2c_stop();

	// read magnetometer values
	i2c_start(); 
	i2c_write_byte(0x3C); // write mag
	i2c_write_byte(0x03); // OUTXH_M, MSB set to enable auto-increment
	i2c_start();		  // repeated start
	i2c_write_byte(0x3D); // read mag
	unsigned char mxh = i2c_read_byte();
	unsigned char mxl = i2c_read_byte();
	unsigned char myh = i2c_read_byte();
	unsigned char myl = i2c_read_byte();
	unsigned char mzh = i2c_read_byte();
	unsigned char mzl = i2c_read_last_byte();
	i2c_stop();

	a->x = axh << 8 | axl;
	a->y = ayh << 8 | ayl;
	a->z = azh << 8 | azl;
	m->x = mxh << 8 | mxl;
	m->y = myh << 8 | myl;
	m->z = mzh << 8 | mzl;
}




// This will print on the serial min and max values of the compass reading
void compass_calibration (void)
{
	vector a={0,0,0};
	vector m={0,0,0};
		
	vector mmin={0,0,0};
	vector mmax={0,0,0};

		
	while(1)
	{
		
		compass_read_data(&a,&m);
		
		
// Mmin handler		
		if(m.x  <  mmin.x)
		mmin.x  =  m.x;
		
		if(m.y  <  mmin.y)
		mmin.y  =  m.y;
		
		if(m.z  <  mmin.z)
		mmin.z  =  m.z;
		
// Mmax handler
		if(m.x  >  mmax.x)
		mmax.x  =  m.x;
		
		if(m.y  >  mmax.y)
		mmax.y  =  m.y;
		
		if(m.z  >  mmax.z)
		mmax.z  =  m.z;	
		
		//Send Mmin values
		USART_Send_string(" MMIN ");  
		USART_Send_string("X: ");
		USART_Send_int((int)mmin.x);
		USART_Send_string(" Y: ");
		USART_Send_int((int)mmin.y);
		USART_Send_string(" Z: ");
		USART_Send_int((int)mmin.z);
		USART_Send_string("    ");
		//Send Mmax values
		USART_Send_string(" MMAX ");  
		USART_Send_string("X: ");
		USART_Send_int((int)mmax.x);
		USART_Send_string(" Y: ");
		USART_Send_int((int)mmax.y);
		USART_Send_string(" Z: ");
		USART_Send_int((int)mmax.z);
		USART_Send_string(" \n");
		
	}

}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
float get_heading(vector *a, vector *m, vector *p)
{
	
	// shift and scale
	m->x = (m->x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
	m->y = (m->y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
	m->z = (m->z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;
	
	
	vector E;
	vector N;

	// cross magnetic vector (magnetic north + inclination) with "down" (acceleration vector) to produce "east"
	vector_cross(m, a, &E);
	vector_normalize(&E);

	// cross "down" with "east" to produce "north" (parallel to the ground)
	vector_cross(a, &E, &N);
	vector_normalize(&N);

	// compute heading
	float heading = round(atan2(vector_dot(&E, p), vector_dot(&N, p)) * 180 / M_PI);
	
	if (heading > 180)
		heading -= 360;
	if (heading < -180)
		heading += 360;
	return heading;
}






void gyro_config(void)
{
 	DDRC = 0;                              // all inputs
	PORTC = (1 << PORTC4) | (1 << PORTC5); // enable pull-ups on SDA and SCL, respectively

	TWSR = 0;  // clear bit-rate prescale bits
	TWBR = 12; // produces an SCL frequency of 400 kHz with a 16 MHz CPU clock speed

	// clear();  

	//enable gyro
	i2c_start(); 
	i2c_write_byte(0xD2); // write gyro
	i2c_write_byte(0x20); // CTRL_REG1_A
	i2c_write_byte(0xBF); // normal power mode, 400 Hz data rate, cut-off 110
	i2c_stop();
	
	// Configures scale
	i2c_start(); 
	i2c_write_byte(0xD2); // write gyro
	i2c_write_byte(0x23); // CTRL_REG4_A
	//Next value is 0x00 for 250dps, 0x10 for 500dps, 0x20 for 2000dps
	i2c_write_byte(0x10); 
	i2c_stop();



}


void gyro_read_data(vector *g)
{
	// read gyroscope values
	i2c_start();
	i2c_write_byte(0xD2); // write acc
	i2c_write_byte(0xa8); // OUT_X_L, MSB set to enable auto-increment
	i2c_start();		  // repeated start
	i2c_write_byte(0xD3); // read acc
	unsigned char gxl = i2c_read_byte();
	unsigned char gxh = i2c_read_byte();
	unsigned char gyl = i2c_read_byte();
	unsigned char gyh = i2c_read_byte();
	unsigned char gzl = i2c_read_byte();
	unsigned char gzh = i2c_read_last_byte();
	i2c_stop();


	g->x = gxh << 8 | gxl;
	g->y = gyh << 8 | gyl;
	g->z = gzh << 8 | gzl;

}