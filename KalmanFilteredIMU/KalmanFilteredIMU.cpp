/*
 * AVRGCC2.cpp
 *
 * Created: 21/12/2011 16:43:10
 * Author: Loïc Kaemmerlen
 * Thanks to Pascal Madesclair for his help

*/


# define F_CPU 16000000L
#include <avr/io.h>			// including the avr IO lib
#include <util/delay.h>		// including the avr delay lib
#include <avr/interrupt.h>	// including the avr interrupt lib
#include <inttypes.h>
#include <stdlib.h> 

#include "usart.h"			// Include the Serial Library
#include "sensors.h"		// Include the Compass Library
#include "KalmanFilter.h"	// Include the Filter Library
#include "mathstools.h"		// Include the Vectors and Matrix Library



///////////////////// DEFINE FUNCTIONS //////////////////

void initialize(void);			// Initialization routines
void init_interrupts(void);		// Timer1 interrupts initialization
void filter(void);				// Kalman filter

//////////////////////!define functions//////////////////






///////////////////// DEFINE VARIABLES //////////////////

// Timer1 reload value. This defines the filtering frequency:
// In init_interrupts(), I set the prescalar to 8, which means there's an interrupt every ~33ms if tcnt1=0
// (CPU frequency) / (TimerCount) / (prescaler value) = 16000000/65536/8 =  0.032768s ~= 33ms  and 1/0.032768 = 30,5Hz

// 0b1000100010111000  is 35000 in decimal. Which means that the 16 bits timer1 provides an interrupt every
// 2^16 - 35000 = 30536 clock cycles.
// So we now have an interrupt every 33/ (65536/30536) ~= 15ms.
// The filtering frequency is then 1000ms/15ms ~= 66,6Hz
unsigned int tcnt1=0b1000100010111000;		

vector anglesGyroGlobal;



//////////////////////!define variables//////////////////


// Globally saved angle states
	vector anglesAccel = {0,0,0};
	vector anglesGyro  = {0,0,0};			
	vector anglesOutput = {0,0,0};	
		
//KALMAN MATRIX DECLARATIONS		
	// Steps and variables names are based on the wikipedia page
	// http://en.wikipedia.org/wiki/Kalman_filter
	
	Matrix xk (3,3);			// Predicted state k at k-1
	Matrix xnew  (3,3);			// New measured state
	Matrix uk  (3,3);			// rate k
	
	
	Matrix I (3,3);				// Identity matrix
	Matrix Pk(3,3);				// Covariance matrix
	Matrix R (3,3);				// Measurement noise (linked to accelerometer)
	Matrix Q (3,3);				// Processing noise (linked to gyroscope)

	Matrix S (3,3); 
	Matrix K (3,3);				// Kalman Gain
	
				

// Pointers to matrices which need to be globally saved
	Matrix*	xkptr = &xk;
	Matrix* Pkptr = &Pk;
	Matrix* Sptr = &S;
	Matrix* Kptr = &K;	
 

 
 int main(void)
 {

	// Initialization
	initialize();				// Global function for all initializations
	anglesGyroGlobal ={0,0,0};	 
	USART_Send_string("Begin\n");
	_delay_ms(100);	
		
		
	//Configure and initalize the sensors
	compass_config();
	gyro_config();
	
	
	// Kalman filter matrix initializations
	Pk.mat[0][0]=1; Pk.mat[1][1]=1; Pk.mat[2][2]=1;
	I.mat[0][0]=1; I.mat[1][1]=1; I.mat[2][2]=1;					// Identity matrix
	
	
	///////////////// COVARIANCE NOISE MATRICES /////////////////
	R.mat[0][0]=0.5; R.mat[1][1]=0.5; R.mat[2][2]=0.01;				// Measurement noise (accelerometer)
	Q.mat[0][0]=0.005;  Q.mat[1][1]=0.005;	Q.mat[2][2]=0.0001;	// Processing noise (gyro)
	
		
	
	///////////////// ///////////// /////////////////
	///////////////// INTERRUPT USE /////////////////
	// Comment to deactivate interrupts, uncomment to work with timer1 interrupts
	 sei();
	///////////////// ///////////// /////////////////
	
	while(1) 
	{
		// Infinite loop so that interrupts can be called
	}
	
 } // End of Main





void filter(void)
{		
		// Get new sensor values
		anglesAccel = accelcompass_angle_acquisition();
		anglesGyro = gyro_angle_acquisition();
		
		// Put those values in the right measurement/processing matrices
		xnew.mat[0][0] = anglesAccel.x; xnew.mat[1][1] = anglesAccel.y; xnew.mat[2][2] = anglesAccel.z;
		uk.mat[0][0] = anglesGyro.y;  uk.mat[1][1] = -anglesGyro.x;  uk.mat[2][2] = -anglesGyro.z;
	
		// Run the filter
		KalmanFilter( xkptr,  xnew,  uk,  Pkptr,  R, Q, Sptr,  Kptr,  I);
		
		// Shift results
		anglesOutput = {xk(0,0), xk(1,1), xk(2,2)};
		if(anglesOutput.x >180)
			{anglesOutput.x-=360;}
		if(anglesOutput.x <-180)
			{anglesOutput.x+=360;}
		if(anglesOutput.y >180)
			{anglesOutput.y-=360;}
		if(anglesOutput.y <-180)
			{anglesOutput.y+=360;}
		if(anglesOutput.z >180)
			{anglesOutput.z-=360;}
		if(anglesOutput.z <-180)
			{anglesOutput.z+=360;}
		
		// Output results in CSV format (Comma-separated values)
		USART_Send_string("000");
		USART_Send_string(",");
		
		printFloat(anglesAccel.x,0);
		USART_Send_string(",");
		
		printFloat(anglesOutput.x,0);
		USART_Send_string(",");
		
		printFloat(anglesAccel.y,0);
		USART_Send_string(",");
		
		printFloat(anglesOutput.y,0);
		USART_Send_string(",");
		
		printFloat(anglesAccel.z,0);
		USART_Send_string(",");
		
		printFloat(anglesOutput.z,0);
		USART_Send_string("\n");
	}		




///////////////////////////
// Timer1 Interrupt handler
    ISR(TIMER1_OVF_vect) 
	{		 
		TCNT1 = tcnt1;		// Reload the timer value
		sei();				// Re-enable the timer as fast as possible
		filter();			// Run the filtering
    }  




// Initialization function
void initialize(void)
{
	init_interrupts();  // Initializes Timer1 interrupts
	USART_Init(MYUBRR); // Initializes the serial communication
}	


// Interrupts configuration
void init_interrupts(void)
{
	  /* First disable the timer overflow interrupt while we're configuring */  
      TIMSK1 &= ~(1<<TOIE1);  
      
      /* Configure timer2 in normal mode (pure counting, no PWM etc.) */  
      TCCR1A &= ~((1<<WGM11) | (1<<WGM10));  
      TCCR1B &= ~((1<<WGM13) | (1<<WGM12));  
      
 
      /* Disable Compare Match A interrupt enable (only want overflow) */  
      TIMSK1 &= ~(1<<OCIE1A);  
      
      // Prescalar = 8.  Time between interrupts ~=32ms;
      TCCR1B &= ~(1<<CS12);			// Clear bit
	  TCCR1B |= (1<<CS11);          // Set bit 
	  TCCR1B &= ~(1<<CS10);			// Clear bit
      
      /* Finally load and enable the timer */  
      TCNT1 = tcnt1;  
      TIMSK1 |= (1<<TOIE1);  
}
 

      
	 