/*
 * usart.h
 *
 * Created: 07/12/2011 15:16:27
 *  Author: Boomber
 */ 


#include <inttypes.h>
#include <stdio.h> // for size_t


#ifndef USART_H_
#define USART_H_


#define FOSC 16000000 // Clock Speed
#define BAUD 115200
#define MYUBRR (((((FOSC * 10) / (16L * BAUD)) + 5) / 10) - 1)

void USART_Init( unsigned int ubrr);

void USART_Sendbyte( unsigned char data );
void USART_Send_string(const char *str);
void USART_Send_int(int d);


unsigned char USART_Receive( void );


// FROM ARDUINO CORE
	void write( unsigned char data );


    void printNumber(unsigned long, uint8_t);
	void printFloat(double, uint8_t);

//    virtual void write_uint(uint8_t) = 0;
     void write_constchar(const char *str);
     void write_constuint(const uint8_t *buffer, size_t size);
    
//    void print(const String &);
    void print_constchar(const char[]);
    void print_char(char, int = 0);
    void print_unsignedchar(unsigned char, int = 10);
    void print_int(int, int = 10);
    void print_unsignedint(unsigned int, int = 10);
    void print_long(long, int = 10);
    void print_unsignedlong(unsigned long, int = 10);
    void print_double(double, int = 2);


 
#endif /* USART_H_ */