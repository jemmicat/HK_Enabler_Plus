/*
 * usart.cpp
 *
 * Created: 07/12/2011 15:17:35
 *  Author: Boomber
 */ 
#include "usart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>			// Conversions
#include <inttypes.h>


void USART_Init( unsigned int ubrr)
{
/*Set baud rate */
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
//Enable receiver and transmitter */
UCSR0B = (1<<RXEN0)|(1<<TXEN0);
/* Set frame format: 8data, 2stop bit */
UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}




void USART_Sendbyte( unsigned char data )
{
/* Wait for empty transmit buffer */
while ( !( UCSR0A & (1<<UDRE0)) )
;
/* Put data into buffer, sends the data */
UDR0 = char(data);
}

void USART_Send_string(const char *str)
{

	  while (*str) 
      USART_Sendbyte(*str++);
	
}

void USART_Send_int(int d )
{
	char str[10];
	sprintf(str,"%d",d);
	USART_Send_string(str);
	
}

unsigned char USART_Receive( void )
{
/* Wait for data to be received */
while ( !(UCSR0A & (1<<RXC0)) )
;
/* Get and return received data from buffer */
return UDR0;
}



// FROM ARDUINO CORE

void write( unsigned char data )
{
	USART_Sendbyte(data);
}



// default implementation: may be overridden
void write_constchar(const char *str)
{
  while (*str)
    write(*str++);
}

// default implementation: may be overridden 
void write_constuint(const uint8_t *buffer, size_t size)
{
  while (size--)
    write(*buffer++);
}

// void Print::print(const String &s)
// {
//   for (int i = 0; i < s.length(); i++) {
//     write(s[i]);
//   }
// }

void print_constchar(const char str[])
{
  write_constchar(str);
}

void print_char(char c, int base)
{
  print_long((long) c, base);
}

void print_unsignedchar(unsigned char b, int base)
{
  print_unsignedlong((unsigned long) b, base);
}

void print_int(int n, int base)
{
  print_long((long) n, base);
}

void print_unsignedint(unsigned int n, int base)
{
  print_unsignedlong((unsigned long) n, base);
}

void print_long(long n, int base)
{
  if (base == 0) {
    write(n);
  } else if (base == 10) {
    if (n < 0) {
      print_constchar("-");
      n = -n;
    }
    printNumber(n, 10);
  } else {
    printNumber(n, base);
  }
}

void print_unsignedlong(unsigned long n, int base)
{
  if (base == 0) write(n);
  else printNumber(n, base);
}

void print_double(double n, int digits)
{
  printFloat(n, digits);
}



// Private Methods /////////////////////////////////////////////////////////////

void printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;

  if (n == 0) {
    print_constchar("0");
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print_char((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

void printFloat(double number, uint8_t digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     print_constchar("-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print_unsignedlong(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print_constchar("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print_int(toPrint);
    remainder -= toPrint; 
  } 
}
