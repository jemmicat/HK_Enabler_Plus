/*
 * i2ccomm.h
 *
 * Created: 16/12/2011 16:52:50
 *  Author: Loïc KAEMMERLEN
 */ 


#ifndef I2CCOMM_H_
#define I2CCOMM_H_


void i2c_start();
void i2c_write_byte(char byte);
char i2c_read_byte();
char i2c_read_last_byte();
void i2c_stop();

#endif /* I2CCOMM_H_ */