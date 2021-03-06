// JCDesigns HK Enabler PLUS Arduino Version 1.0.5
// Uses Sparkfun Arduino Pro Micro (Or equivalent)https://www.sparkfun.com/products/11098
// Uses TH3122.4 chip Sourced from http://www.ebay.com/itm/2x-Melexis-TH3122-4-IBUS-I-Bus-K-Bus-Transceiver-/221296386461?pt=LH_DefaultDomain_0&hash=item33864ae19d
// Uses PC Board Sourced From OSH Park (3 boards for $5 sq in)
// connections for 10DOF(axis) IMU L3G4200D+ADXL345+HMC5883L+BMP085 (3V-5V compatible)sensor module http://www.ebay.com/itm/141096879704?ssPageName=STRK:MEWNX:IT&_trksid=p3984.m1497.l2649
// IMU Also here https://core-electronics.com.au/store/index.php/10dof-imu-module-with-l3g4200d-adxl345-hmc5883l-bmp085-gy-80.html 
// Holes For attching https://www.sparkfun.com/products/9394
// Send Patterns from the I-Bus/K-Bus for Harman Kardon Radio Emulation 
// based on https://github.com/blalor/iPod_IBus_adapter
// Idea From http://www.northamericanmotoring.com/forums/electrical/155161-getting-more-out-of-the-r53-mfsw.html
// With  help from http://www.gbmini.net/wp/2004/04/running_harmon_kardon_with_no_factory_head_unit/



// SEN/STA of TH3122 connected to digital pin 2 (interrupt 1)
// if SEN/STA is low, there is traffic on the bus
// forcing SEN/STA low clears line for transmission
// buttons to change sound mode can be added at digital pins
// if not interfacing steering wheel buttons to radio, they can be mapped to digital pins to control things

#include <EEPROM.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <serLCD.h>
#include <L3G4200D.h>
#include <HMC5883L.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_ADXL345.h>
#include <Adafruit_Sensor.h>


///////////////////////////////////////////////////////////////////////////
// Pin/Button connections...
//PCB has connections for 4 buttons at D2, D4, D5, and D6
#define DSP_mode    14
#define SENSTA      4 //if SEN/STA is jumpered to Ground, D4 is open  
#define Volume_plus 15
#define Fader_front 16
#define Fader_rear  10

/*
 * LEDs:
 *   red: missed poll from radio
 *   yellow: processing incoming IBus data
 *   green: sending IBus data
 *   
 *   yellow + green: contention/collision when sending
 */
#define LED_RED      5 // red
#define LED_ORANGE  6 // orange

#define LED_GREEN  7 // green

//i2C connections for IMU
#define i2C_SDA 2 
#define i2C_SCL 3

SoftwareSerial LCD(9, 8); // RX, TX for LCD screen


/////////////////////////////////////////////////////////////

#define MAX_EXPECTED_LEN 64

#define TX_BUF_LEN 80
#define RX_BUF_LEN (MAX_EXPECTED_LEN + 2)
#define CONTENTION_TIMEOUT 173

// addresses of IBus devices
#define RAD_ADDR  0x68
#define DSP_ADDR  0x6A
#define SW_ADDR   0x50

// static offsets into the packet
#define PKT_SRC  0
#define PKT_LEN  1
#define PKT_DEST 2
#define PKT_CMD  3

boolean Radio_turned_on; 
boolean Radio_ready;
int packet_delay = 20;
int retry_delay = 5;
int traffic = 0;

// buffer for building outgoing packets
uint8_t tx_buf[TX_BUF_LEN];

// buffer for processing incoming packets; same size as serial buffer
uint8_t rx_buf[RX_BUF_LEN];

// trigger time to turn off LED
unsigned long ledOffTime; // 500ms interval

// timeout duration before giving up on a read
unsigned long readTimeout; // variable


void setup() { 
  pinMode (SENSTA, INPUT);
  pinMode (DSP_mode,INPUT_PULLUP);
  pinMode (Fader_front, INPUT_PULLUP);
  pinMode (Fader_rear, INPUT_PULLUP);
  pinMode (Volume_plus, INPUT_PULLUP);
  pinMode (LED_RED, OUTPUT);  
  pinMode (LED_ORANGE, OUTPUT);
  pinMode (LED_GREEN, OUTPUT);
  
  Radio_turned_on = false; 
  Radio_ready = false;
  
  
  
  // set up serial for IBus; 9600,8,E,1 I set these up directly as they didn't seem to work right using Arduino call out
    Serial1.begin(9600);
    UCSR1C |= (1 << UPM11);
    UCSR1C &= ~(1 << UPM10);
    UCSR1C |= (0 << USBS1);
    
  //start recieving Serial Data 
    Serial.begin(9600);
   
  //LCD Serial
   LCD.begin(9600);
  
  //toggleSplash();
  //turnDisplayOn();
  //backlight(157);// 128 = OFF, 157 = Fully ON, everything inbetween = varied brightnbess 
 
   
   
} 

void loop() {  
    LCD.print("Hello"); 
    Send_HK_Codes();

}  

//-------------------------------------------------------------------------------------------
void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCD.write(0xFE);
  LCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
void selectLineOne()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(128); //position
}
//-------------------------------------------------------------------------------------------
void selectLineTwo()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(192); //position
}
//-------------------------------------------------------------------------------------------
void moveCursorRightOne()
{
  //moves the cursor right one space
  LCD.write(0xFE); //command flag
  LCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void moveCursorLeftOne()
{
  //moves the cursor left one space
  LCD.write(0xFE); //command flag
  LCD.write(16); // 0x10
}
//-------------------------------------------------------------------------------------------
void scrollRight()
{
  //same as moveCursorRightOne
  LCD.write(0xFE); //command flag
  LCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void scrollLeft()
{
  //same as moveCursorLeftOne
  LCD.write(0xFE); //command flag
  LCD.write(24); // 0x18
}
//-------------------------------------------------------------------------------------------
void turnDisplayOff()
{
  //this tunrs the display off, but leaves the backlight on. 
  LCD.write(0xFE); //command flag
  LCD.write(8); // 0x08
}
//-------------------------------------------------------------------------------------------
void turnDisplayOn()
{
  //this turns the dispaly back ON
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void underlineCursorOn()
{
  //turns the underline cursor on
  LCD.write(0xFE); //command flag
  LCD.write(14); // 0x0E
}
//-------------------------------------------------------------------------------------------
void underlineCursorOff()
{
  //turns the underline cursor off
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void boxCursorOn()
{
  //this turns the box cursor on
  LCD.write(0xFE); //command flag
  LCD.write(13); // 0x0D
}
//-------------------------------------------------------------------------------------------
void boxCursorOff()
{
  //this turns the box cursor off
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void toggleSplash()
{
  //this toggles the spalsh screenif off send this to turn onif on send this to turn off
  LCD.write(0x7C); //command flag = 124 dec
  LCD.write(9); // 0x09
}
//-------------------------------------------------------------------------------------------
int backlight(int brightness)// 128 = OFF, 157 = Fully ON, everything inbetween = varied brightnbess 
{
  //this function takes an int between 128-157 and turns the backlight on accordingly
  LCD.write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
  LCD.write(brightness); // any value between 128 and 157 or 0x80 and 0x9D
}
//-------------------------------------------------------------------------------------------
void scrollingMarquee()
{
//This function scroll text across the screen on both lines
  clearScreen(); // it's always good to clear the screen before movonh onto a new print
  for(int j = 0; j < 17; j++)
  {
    selectLineOne();
    for(int i = 0; i < j;i++)
      moveCursorRightOne();
    LCD.print("SPARK");
    selectLineTwo();
    for(int i = 0; i < j;i++)
      moveCursorRightOne();
    LCD.print(" FUN");
    delay(500); // you must have a delay, otherwise the screen will print and clear before you can see the text
    clearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void counter()
{
  //this function prints a simple counter that counts to 10
  clearScreen();
  for(int i = 0; i <= 10; i++)
  {
    LCD.print("Counter = ");
    LCD.print(i, DEC);
    delay(500);
    clearScreen();
  }
}



