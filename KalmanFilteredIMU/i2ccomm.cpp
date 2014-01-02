/*
 * i2ccomm.cpp
 *
 * Created: 16/12/2011 16:52:58
 *  Author: Loïc KAEMMERLEN
 */ 

#include "i2ccomm.h"

#include <avr/io.h>  
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>	

void i2c_start() {  
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // send start condition  
	while (!(TWCR & (1 << TWINT)));  
}  
  
void i2c_write_byte(char byte) {  
	TWDR = byte;              
	TWCR = (1 << TWINT) | (1 << TWEN); // start address transmission  
	while (!(TWCR & (1 << TWINT)));  
}  
  
char i2c_read_byte() {  
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN); // start data reception, transmit ACK  
	while (!(TWCR & (1 << TWINT)));  
	return TWDR;  
}  

char i2c_read_last_byte() {  
	TWCR = (1 << TWINT) | (1 << TWEN); // start data reception
	while (!(TWCR & (1 << TWINT)));  
	return TWDR;  
}  
  
void i2c_stop() {  
	  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send stop condition  
}  