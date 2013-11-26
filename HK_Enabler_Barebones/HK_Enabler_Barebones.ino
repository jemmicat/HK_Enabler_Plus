// JCDesigns HK Enabler Arduino Version 1.0
// Uses Sparkfun Arduino Pro Micro (Or equivalent)https://www.sparkfun.com/products/11098
// Uses TH3122.4 chip Sourced from http://www.ebay.com/itm/2x-Melexis-TH3122-4-IBUS-I-Bus-K-Bus-Transceiver-/221296386461?pt=LH_DefaultDomain_0&hash=item33864ae19d
// Uses PC Board Sourced From OSH Park (3 boards for $5 sq in)

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

// Pin/Button connections...
//PCB has connections for 4 buttons at D2, D4, D5, and D6
#define DSP_mode    2
#define SENSTA      3
#define Volume_plus 4
#define Fader_front 5
#define Fader_rear  6

/*
 * LEDs:
 *   red: missed poll from radio
 *   yellow: processing incoming IBus data
 *   green: sending IBus data
 *   
 *   yellow + green: contention/collision when sending
 */
#define LED_ERR      10 // red
#define LED_IBUS_RX  16 // yellow
#define LED_IBUS_TX  14 // green

#define MAX_EXPECTED_LEN 64

#define TX_BUF_LEN 80
#define RX_BUF_LEN (MAX_EXPECTED_LEN + 2)

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
int packet_delay = 30;

// buffer for building outgoing packets
uint8_t tx_buf[TX_BUF_LEN];

// buffer for processing incoming packets; same size as serial buffer
uint8_t rx_buf[RX_BUF_LEN];

/*
 * time-keeping variables
 */

// trigger time to turn off LED
unsigned long ledOffTime; // 500ms interval

// timeout duration before giving up on a read
unsigned long readTimeout; // variable


void setup() { 
  pinMode (SENSTA, OUTPUT);
  pinMode (DSP_mode,INPUT_PULLUP);
  pinMode (Fader_front, INPUT_PULLUP);
  pinMode (Fader_rear, INPUT_PULLUP);
  pinMode (Volume_plus, INPUT_PULLUP);  
  
  Radio_turned_on = false; 
  Radio_ready = false;
          
  // set up serial for IBus; 9600,8,E,1 I set these up directly as they didn't seem to work right using Arduino call out
    Serial1.begin(9600);
    UCSR1C |= (1 << UPM11);
    UCSR1C &= ~(1 << UPM10);
    UCSR1C |= (0 << USBS1);
    
  //start recieving Serial Data 
    Serial.begin(9600); 
   
} 

void loop() {  
      
  if (Radio_turned_on == false){
    Send_radio_broadcast();
    Radio_turned_on = true;
  }
  
  while ((Radio_turned_on == true) && (Radio_ready == false)){       
  process_incoming_data();
  }
  
  while ((Radio_turned_on == true) && (Radio_ready == true)){       
  ;
  }
 
} 

// {{{ process_incoming_data
boolean process_incoming_data() {
    /*
    The serial reading is pretty naive. If we start reading in the middle of
    a packet transmission, the checksum validation will fail. All data
    received up to that point will be lost. It's expected that this loop will
    eventually synchronize with the stream during a lull in the conversation,
    where all available and "invalid" data will have been consumed.
    */
    
    boolean found_message = false;
    
    uint8_t bytes_availble = Serial1.available();
    
    if (bytes_availble) {
        digitalWrite(LED_IBUS_RX, HIGH);
    }
    
    // filter out packets from sources we don't care about
    // I don't like this solution at all, but until I can implement a timer to 
    // reset in the RX interrupt I think this will at least avoid getting 
    // stuck waiting for enough data to arrive
    if (bytes_availble && (Serial1.peek(PKT_SRC) != DSP_ADDR) && (Serial1.peek(PKT_SRC) != SW_ADDR)) {
        //DEBUG_PGM_PRINTLN("[IBus] dropping byte from unknown source");
        Serial1.remove(1);
    }
    // need at least two bytes to a packet, src and length
    else if (bytes_availble > 2) {
        // length of the data
        uint8_t data_len = Serial1.peek(PKT_LEN);
        
        if (
            (data_len == 0)                || // length cannot be zero
            (data_len >= MAX_EXPECTED_LEN)    // we don't handle messages larger than this
        ) {
            //DEBUG_PGM_PRINTLN("[IBus] invalid packet length");     
            Serial1.remove(1);
        }
        else {
            // length of entire packet including source and length
            uint8_t pkt_len = data_len + 2;

            // index of the checksum byte
            uint8_t chksum_ind = pkt_len - 1;      

            // ensure we've got enough data in the buffer to comprise a
            // complete packet
            if (bytes_availble >= pkt_len) {
                // yep, have enough data
                readTimeout = 0;

                // verify the checksum
                int calculated_chksum = 0;
                for (int i = 0; i < chksum_ind; i++) {
                    calculated_chksum ^= Serial1.peek(i);
                }

                if (calculated_chksum == Serial1.peek(chksum_ind)) {
                    found_message = true;
                    
                    // valid checksum

                    // read packet into buffer and decode
                    for (int i = 0; i < pkt_len; i++) {
                        rx_buf[i] = Serial1.read();                 
                    }                   
                    decode_packet(rx_buf);
                }
                else {
                    // invalid checksum; drop first byte in buffer and try
                    // again
                    //DEBUG_PGM_PRINTLN("[IBus] invalid checksum");
                    Serial1.remove(1);
                }
            } // if (bytes_availble …)
            else {
                // provide a timeout mechanism; expire if needed bytes 
                // haven't shown up in the expected time.
                
                if (readTimeout == 0) {
                    // (10 bits/byte) => 1.042ms/byte; add a 20% fudge factor
                    readTimeout = ((125 * ((pkt_len - bytes_availble) + 1)) / 100);
                    readTimeout += millis();
                }
                else if (millis() > readTimeout) {
                    readTimeout = 0;
                    Serial1.remove(1);
                }
            }
        }
    } // if (bytes_availble  >= 2)
    
    digitalWrite(LED_IBUS_RX, LOW);

    return found_message;
}
// }}}


void decode_packet(const uint8_t *packet) {
    // determine if the packet is from the DSP and addressed to radio or if there
    // are any other packets we should use as a trigger.
     
      if ((packet[PKT_SRC] == DSP_ADDR) && (packet[PKT_DEST] == RAD_ADDR)) {
        
        if (packet[PKT_CMD] == 0x02) {
            // device status ready
            
            // use this as a trigger to send our initial announcment.
            // @todo read up on IBus protocol to see when I should really send 
            // my announcements
            
            //DEBUG_PGM_PRINTLN("[IBus] sending SDRS announcement because radio sent device status ready");
            
            // send SDRS announcement
            Serial.println("DSP Responded");
            Send_radio_settings();
        }
    }
}
// }}}

// {{{ send_radio settings
void Send_radio_settings() {
      
    Send_initialize_begin();
    delay(packet_delay); 
    Send_bass_level();
    delay(packet_delay); 
    Send_treble_level();
    delay(packet_delay); 
    Send_fader_value();
    delay(packet_delay);
    Send_balance_value();
    delay(packet_delay); 
    Send_radio_mode();
    delay(packet_delay);
    Send_dsp_mode();
    delay(packet_delay);
    Send_driver_mode();
    delay(packet_delay);
    Send_initialize_end();
    delay(packet_delay);
    Send_volume();
    Radio_ready = true;    
}
// }}}

// {{{ send_raw_ibus_packet_P
void send_raw_ibus_packet_P(PGM_P pgm_data, size_t pgm_data_len) {
    for (uint8_t i = 0; i < pgm_data_len; i++) {
        tx_buf[i] = pgm_read_byte(&pgm_data[i]);
    }
    
    send_raw_ibus_packet(tx_buf, pgm_data_len);
}
// }}}

// {{{ 
boolean send_raw_ibus_packet(uint8_t *data, size_t data_len) {
    boolean sent_successfully = false;
       
    digitalWrite(LED_IBUS_TX, HIGH);
    ledOffTime = millis() + 500L;
    digitalWrite(LED_IBUS_RX, LOW);   
    Serial1.write(data, data_len);
    sent_successfully = true;
    return sent_successfully;
}


void Send_radio_broadcast(){
  send_raw_ibus_packet_P(PSTR("\x68\x04\xFF\x02\x04\x95"), 6);
  Serial.println("Broadcast Message Sent");
}

void Send_initialize_begin(){  
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xAF\x9F"), 6);
  Serial.println("Initialize Sent");
}

void Send_bass_level(){ 
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\x60\x50"), 6); //center
  Serial.println("Bass Level Sent");
}

void Send_treble_level(){ 
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xC0\xF0"), 6); //center
  Serial.println("Treble Level Sent");
}

void Send_fader_value(){  
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\x80\xB0"), 6); //center
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\x94\xA4"), 6); //I prefer fader biased to the back slightly
  Serial.println("Fader Value Sent");
} 
 
void Send_balance_value(){ 
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\x40\x70"), 6); //center
  Serial.println("Balance Value Sent");
} 

void Send_radio_mode(){  
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xE4\xD4"), 6); //Spatial Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xE3\xD3"), 6); //miniHK Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xE5\xD5"), 6); //Electronic Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xE6\xD6"), 6); //Instrumental Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xE7\xD7"), 6); //Festival Mode
  Serial.println("Radio Mode Sent");
} 

void Send_dsp_mode(){  
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x34\x09\x3B"), 6); //Spatial Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x34\x08\x3A"), 6); //miniHK Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x34\x0A\x3C"), 6); //Electronic Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x34\x0B\x3D"), 6); //Instrumental Mode
  //send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x34\x0C\x3E"), 6); //Festival Mode
  Serial.println("DSP Mode Sent");
}  
  
void Send_driver_mode(){ 
  //the radio always sends a “34 90″ code when the amplifier mode is changed.
  send_raw_ibus_packet_P(PSTR("\x68\x05\x6A\x34\x90\x00\xA3"), 7); //Driver Mode On
  //send_raw_ibus_packet_P(PSTR("\x68\x05\x6A\x34\x91\x00\xA4"), 7); //Driver Mode On
  Serial.println("Driver Mode Sent");
}

void Send_initialize_end(){
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x36\xA1\x91"), 6);
  Serial.println("Initialize End Sent");
}
 
void Send_volume(){
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x32\xF1\xC5"), 6); //volume +15
  delay(packet_delay);
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x32\xF1\xC5"), 6); //volume +15
  delay(packet_delay);
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x32\xF1\xC5"), 6); //volume +15
  delay(packet_delay);
  send_raw_ibus_packet_P(PSTR("\x68\x04\x6A\x32\x31\x05"), 6); //volume +3
  Serial.println("Volume Sent");
}
// }}} 




/* **********************REFERENCE*************************************

  //HK Radio Definitions 
Key_on[] = {0x80,0x04,0xBF,0x11,0x01,0x2B}; 
Key_off[] = {0x80,0x04,0xBF,0x11,0x00,0x2A}; 
Radio_on[] = {0x68,0x04,0xFF,0x02,0x04,0x95}; 
DSP_present[] = {0x6A,0x04,0x68,0x02,0x01,0x05}; 
Initialize_begin[] = {0x68,0x04,0x6A,0x36,0xAF,0x9F}; 
Initialize_end[] = {0x68,0x04,0x6A,0x36,0xA1,0x91}; 
Bass_0[] = {0x68,0x04,0x6A,0x36,0x60,0x50}; 
Treble_0[] = {0x68,0x04,0x6A,0x36,0xC0,0xF0}; 
Fade_rear4[] = {0x68,0x04,0x6A,0x36,0x94,0xA4}; 
Balance_center[] = {0x68,0x04,0x6A,0x36,0x40,0x70}; 
 Radio_spatial[] = {0x68,0x04,0x6A,0x36,0xE4,0xD4};
Radio_miniHK[] = {0x68,0x04,0x6A,0x36,0xE3,0xD3};
Radio_electronic[] = {0x68,0x04,0x6A,0x36,0xE5,0xD5};
Radio_instrumental[] = {0x68,0x04,0x6A,0x36,0xE6,0xD6};
Radio_festival[] = {0x68,0x04,0x6A,0x36,0xE7,0xD7};
Mode_spatial[] = {0x68,0x04,0x6A,0x34,0x09,0x3B}; 
Mode_miniHK[] = {0x68,0x04,0x6A,0x34,0x08,0x3A}; 
Mode_electronic[] = {0x68,0x04,0x6A,0x34,0x0A,0x3C}; 
Mode_instrumental[] = {0x68,0x04,0x6A,0x34,0x0B,0x3D}; 
Mode_festival[] = {0x68,0x04,0x6A,0x34,0x0C,0x3E}; 
Driver_mode_off[] = {0x68,0x05,0x6A,0x34,0x90,0x00,0xA3}; 
Driver_mode_on[] = {0x68,0x05,0x6A,0x34,0x91,0x00,0xA3}; 
Volume_increase15[] = {0x68,0x04,0x6A,0x32,0xF1,0xC5}; 
Volume_increase3[] = {0x68,0x04,0x6A,0x32,0x31,0x05};
Volume_increase6[] = {0x68,0x04,0x6A,0x32,0x61,0x55};
Volume_increase1[] = {0x68,0x04,0x6A,0x32,0x01,0x55}; // check this

//Steering Wheel Controls
SW_Volume_up_press[] = {0x50,0x04,0x68,0x32,0x11,0x1F}; //Volume up = top right button 
SW_Volume_up_release[] = {0x50,0x04,0x68,0x32,0x31,0x3F}; //<+> release
SW_Volume_down_press[] = {0x50,0x04,0x68,0x32,0x10,0x1E}; // Volume down = bottom right button
SW_Volume_down_release[] = {0x50,0x04,0x68,0x32,0x30,0x3E}; // <-> release
SW_Mode_press[] = {0x50,0x04,0x68,0x3B,0x02,0x05}; // Mode press = middle right button 
SW_Mode_release[] = {0x50,0x04,0x68,0x3B,0x22,0x25}; // Mode release 

SW_Next_press[] = {0x50,0x04,0x68,0x3B,0x01,0x06}; // Next track press = top left button 
SW_Next_release[] = {0x50,0x04,0x68,0x3B,0x21,0x26}; // Next track release 
SW_Previous_press[] = {0x50,0x04,0x68,0x3B,0x08,0x0F}; // Previous track press = bottom left button 
SW_Previous_release[] = {0x50,0x04,0x68,0x3B,0x28,0x2F}; // Previous track release 
SW_LM_press[] = {0x50,0x04,0x68,0x3B,0x80,0x87}; // Left middle button press 
SW_LM_release[] = {0x50,0x04,0x68,0x3B,0xA0,0xA7}; // Left middle button release

 With Help From http://www.northamericanmotoring.com/forums/electrical/155161-getting-more-out-of-the-r53-mfsw.html
 With  help from http://www.gbmini.net/wp/2004/04/running_harmon_kardon_with_no_factory_head_unit/
 
 TECHNICAL INFO:
 Harmon Kardon / Radio communications in 2003 MINI Cooper S.
 ===============================================
 When power is applied, the radio sends a broadcast message â€“ presumably indicates that it is turned on.
 68 04 FF 02 04 95 Radio->all …
 Next the DSP sends a message to the radio â€“ presumably indicating that it is present.
 6A 04 68 02 01 05 DSP->Radio …
 
 So now the radio knows it needs to operate in “Harmon Kardon” mode. In this mode the front speaker outputs are fixed at a medium volume level, with no bass/treble/fade/balance applied. Instead the audio control settings are transmitted to the amplifier and are applied to the incoming signal there. The radio sends an initial sequence of settings:
 68 04 6A 36 AF 9F Radio->DSP “initialize begin?”
 68 04 6A 36 60 50 Radio->DSP “bass=center”
 68 04 6A 36 C0 F0 Radio->DSP “treble=center”
 68 04 6A 36 80 B0 Radio->DSP “fade=center”
 68 04 6A 36 40 70 Radio->DSP “balance=center”
 68 04 6A 36 E4 D4 Radio->DSP ????
 68 04 6A 34 09 3B Radio->DSP “mode=spatial”
 68 05 6A 34 90 00 A3 Radio->DSP “driver mode off”
 68 04 6A 36 A1 91 Radio->DSP “initialize end?”
 The values for bass, treble, fade and balance are the current settings â€“ not necessarily “center”. The “36 AF” message at the beginning perhaps sets some flags; this message stops the amplifier putting out any sound, because the radio sends it when it is turned off. It probably also sets all the audio controls to default, and sets the volume to zero. The “36 A1″ at the end presumably allows the amplifier to start running.
 
 The radio now sends a sequence to set the volume. This is set with one or more messages that increase the volume by between 1 and 15 steps:
 68 04 6A 32 F1 C5 Radio->DSP “volume+15″
 68 04 6A 32 F1 C5 Radio->DSP “volume+15″
 68 04 6A 32 31 05 Radio->DSP “volume+3″
 So the volume is now at “33″; the maximum volume that can be sent by the radio is “63″.
 
 The radio / amplifier system is now operating.
 
 The volume can be adjusted up/down. The radio sends messages to increase or decrease the volume by 1-15 steps; it must keep track of how many steps are needed:
 68 04 6A 32 10 24 Radio->DSP “volume-1″
 68 04 6A 32 40 74 Radio->DSP “volume-4″
 68 04 6A 32 10 24 Radio->DSP “volume-1″
 68 04 6A 32 30 04 Radio->DSP “volume-3″
 68 04 6A 32 11 25 Radio->DSP “volume+1″
 68 04 6A 32 61 55 Radio->DSP “volume+6″
 68 04 6A 32 11 25 Radio->DSP “volume+1″
 68 04 6A 32 51 65 Radio->DSP “volume+5″
 After this sequence, the volume would be at “36″.
 
 The audio controls can also be adjusted; bass/treble/fade/balance are all adjusted using the “36 xy” code where the “x” part indicates the audio function and the “y” part indicates the setting:
 36 60 “bass=center”
 36 6y “bass=boost”; y=2/4/6/8/A/C
 36 7y “bass=cut”; y=2/4/6/8/A/C
 36 C0 “treble=center”
 36 Cy “treble=boost”; y=2/4/6/8/A/C
 36 Dy “treble=cut”; y=2/4/6/8/A/C
 36 80 “fade=center”
 36 8y “fade=front?”; y=1/2/3/4/5/6/8/A/F
 36 9y “fade=rear?”; y=1/2/3/4/5/6/8/A/F
 36 40 “balance=center”
 36 4y “balance=left?”; y=1/2/3/4/5/6/8/A/F
 36 5y “balance=right?”; y=1/2/3/4/5/6/8/A/F
 I would guess that all the audio controls can vary to “F”=15, so that it might be possible to get more bass/treble boost by sending a “36 6F” or “36 CF” code.
 
 The amplifier mode can also be adjusted using the “34 0y” code where the “y” part selects the mode:
 34 08 “MINI H/K”
 34 09 “SPATIAL”
 34 0A “ELECTRONIC”
 34 0B “INSTRUMENTAL”
 34 0C “FESTIVAL”
 Other values of “y” seem to have no effect.
 
 The “driver mode” can also be adjusted on or off:
 34 90 “DRIVER MODE OFF”
 34 91 “DRIVER MODE ON”
 Some modes do not allow the driver mode to be turned on; the radio always sends a “34 90″ code when the amplifier mode is changed.
 Turning driver mode on when it’s not allowed seems to have no effect.
 
 If the radio is turned off, a “final” message is sent to the amplifier:
 68 04 6A 36 AF 9F Radio->DSP “initialize begin?”
 This silences the amplifier even if it is not powered off! Strange, since the radio also sends a voltage to control the amplifier.

 ---------------------------------------------------------------------------------
 Steering Wheel ------------------------------------------------------------------    
 ---------------------------------------------------------------------------------

MINI with the 2 spoke steering wheel 

50 04 68 32 11 1F Volume up = top right button 
50 04 68 32 31 3F <+> release
50 04 68 32 10 1E Volume down = bottom right button
50 04 68 32 30 3E <-> release

50 04 68 3B 01 06 Next track press = top left button 
50 04 68 3B 21 26 Next track release 

50 04 68 3B 08 0F Previous track press = bottom left button 
50 04 68 3B 28 2F Previous track release 
 
50 04 68 3B 02 05 Mode press = middle right button 
50 04 68 3B 22 25 Mode release 

50 04 68 3B 80 87 Left middle button press 
50 04 68 3B A0 A7 Left middle button release

 */

