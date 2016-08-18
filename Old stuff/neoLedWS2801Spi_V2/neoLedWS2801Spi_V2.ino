/*
 * PixelInvaders serial-led-gateway, Copyright (C) 2012 michael vogt <michu@neophob.com>
 * Tested on Teensy and Arduino
 * 
 * ------------------------------------------------------------------------
 *
 * This is the SPI version, unlike software SPI which is configurable, hardware 
 * SPI works only on very specific pins. 
 *
 * On the Arduino Uno, Duemilanove, etc., clock = pin 13 and data = pin 11. 
 * For the Arduino Mega, clock = pin 52, data = pin 51. 
 * For the ATmega32u4 Breakout Board and Teensy, clock = pin B1, data = B2. 
 *
 * ------------------------------------------------------------------------
 *
 * This file is part of PixelController.
 *
 * PixelController is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * PixelController is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * 	
 */

//#include <FastSPI_LED.h>
#include <FastSPI_LED2.h>


// ======= START OF USER CONFIGURATION =======

//define nr of Panels*2 here, 4 means 2 panels
#define NR_OF_PANELS 12

// ======= END OF USER CONFIGURATION ======= 

#define PIXELS_PER_PANEL 32
#define NUM_LEDS (NR_OF_PANELS*PIXELS_PER_PANEL)

//to draw a frame we need arround 20ms to send an image. the serial baudrate is
//NOT the bottleneck. 
#define BAUD_RATE 115200

//--- protocol data start
#define CMD_START_BYTE 0x01
#define CMD_SENDFRAME 0x03
#define CMD_PING  0x04

#define START_OF_DATA 0x10 
#define END_OF_DATA 0x20

//frame size for specific color resolution
//32pixels * 2 byte per color (15bit - one bit wasted)
#define COLOR_5BIT_FRAME_SIZE 64
#define SERIAL_HEADER_SIZE 5
#define SERIAL_FOOTER_SIZE 1
#define SERIAL_PACKET_SIZE (COLOR_5BIT_FRAME_SIZE+SERIAL_HEADER_SIZE+SERIAL_FOOTER_SIZE)
//--- protocol data end

//8ms is the minimum! else we dont get any data!
#define SERIAL_DELAY_LOOP 4
#define SERIAL_WAIT_DELAY 8

//this should match RX_BUFFER_SIZE from HardwareSerial.cpp
//array that will hold the serial input string
byte serInStr[COLOR_5BIT_FRAME_SIZE+SERIAL_HEADER_SIZE]; 	 				 

#define SERIALBUFFERSIZE 4
byte serialResonse[SERIALBUFFERSIZE];

byte g_errorCounter;

int jj=0,k=0;
byte serialDataRecv;
int g=0;

/*
// Sometimes chipsets wire in a backwards sort of way
struct CRGB { 
  unsigned char b; 
  unsigned char r; 
  unsigned char g; 
};
// struct CRGB { unsigned char r; unsigned char g; unsigned char b; };
struct CRGB *leds;
*/

CRGB leds[NUM_LEDS];



// --------------------------------------------
//     send status back to library
// --------------------------------------------
static void sendAck() {
  serialResonse[0] = 'A';
  serialResonse[1] = 'K';
  serialResonse[2] = Serial.available();
  serialResonse[3] = g_errorCounter;
  Serial.write(serialResonse, SERIALBUFFERSIZE);

#if defined (CORE_TEENSY_SERIAL)
  //Teensy supports send now
  Serial.send_now();
#endif
}


// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b) {
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if (WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170; 
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

// --------------------------------------------
//     do some animation until serial data arrives
// --------------------------------------------
void rainbow() {
  //Serial.print("8");
  delay(1);
  
  
  k++;
  if (k>50) {
    k=0;
    jj++;
    if (jj>255) {
      jj=0; 
    }

    for (int j = 0; j < 3; j++) { 
      for (int i = 0 ; i < NUM_LEDS; i++ ) {

        uint32_t color = Wheel( (i + jj) % 255);
        leds[i].r = (color>>16)&255;
        leds[i].g = (color>>8)&255; 
        leds[i].b = color&255; 

      }
    }
    
    //FastSPI_LED.show();
    LEDS.show();
    
  }
}

void line() {
  delay(10);
  
  k++;
  if (k>50) {
    k=0;
    jj++;
    if (jj>255) {
      jj=0; 
    }

    for (int j = 0; j < 3; j++) { 

      for (int i = 0 ; i < NUM_LEDS; i++ ) {

        uint32_t color = Wheel( (i + jj) % 255);
        leds[i].r = 0;
        leds[i].g = 0; 
        leds[i].b = 0; 
        }
      
      g++;    
      uint32_t color = Wheel( jj % 255);
      leds[g].r = (color>>16)&255;
      leds[g].g = (color>>8)&255; 
      leds[g].b = color&255; 
      
      if (g >= NUM_LEDS) {
      g = 0;
      }    
    }
    
    //FastSPI_LED.show();
    LEDS.show();
    
  }
}


// --------------------------------------------
//      setup
// --------------------------------------------
void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  
  memset(serialResonse, 0, SERIALBUFFERSIZE);

  //im your slave and wait for your commands, master!
  Serial.begin(BAUD_RATE); //Setup high speed Serial
  Serial.flush();
  
  //FastSPI_LED.setLeds(NUM_LEDS);
  //FastSPI_LED.setChipset(CFastSPI_LED::SPI_WS2801);

  //LEDS.addLeds<WS2801, 11, 13, BRG, DATA_RATE_MHZ(1)>(leds, NUM_LEDS);
  LEDS.addLeds<WS2801, BRG>(leds, NUM_LEDS);
  //Could be useful to limit the SPI speed
  //Put ws2801 strip on the hardware SPI pins with a BGR ordering of rgb and limited to a 1Mhz data rate
  //LEDS.addLeds<WS2801, 11, 13, BGR, DATA_RATE_MHZ(1)>(leds, NUM_LEDS);

  //select spi speed, 7 is very slow, 0 is blazing fast
//  FastSPI_LED.setDataRate(1);
//  FastSPI_LED.init();
//  FastSPI_LED.start();
//  leds = (struct CRGB*)FastSPI_LED.getRGBData(); 

  rainbow();      // display some colors

  serialDataRecv = 0;   //no serial data received yet  
}

// --------------------------------------------
//      main loop
// --------------------------------------------
void loop() {
  g_errorCounter=0;

  // see if we got a proper command string yet, 0 means no data read
  if (readCommand(serInStr) == 0) {
    //nope, nothing arrived yet...
    if (serialDataRecv==0) { //if no serial data arrived yet, show the rainbow...
      rainbow();    	
    }
    return;
  }

  //led offset
  byte ofs    = serInStr[1];
  //how many bytes we're sending
  byte sendlen = serInStr[2];
  //what kind of command we send
  byte type = serInStr[3];
  //get the image data
  byte* cmd    = serInStr+5;

  switch (type) {
  case CMD_SENDFRAME:
    //the size of an image must be exactly 64bytes for 8*4 pixels
    if (sendlen == COLOR_5BIT_FRAME_SIZE) {
      updatePixels(ofs, cmd);
    } 
    else {
      g_errorCounter=100;
    }
    break;

  case CMD_PING:
    //just send the ack!
    serialDataRecv = 1; 
    sendAck();    
    //Serial.print("1");
    break;

    // case CMD_CONNECTION_CLOSED:
    //   serialDataRecv = 0;        
    //   break;

  default:
    //invalid command
    g_errorCounter=130; 
    break;
  }

  //send ack to library - command processed
  //sendAck();
}


//convert a 15bit color value into a 24bit color value
uint32_t convert15bitTo24bit(uint16_t col15bit) {
  uint8_t r=col15bit & 0x1f;
  uint8_t g=(col15bit>>5) & 0x1f;
  uint8_t b=(col15bit>>10) & 0x1f;

  return Color(r<<3, g<<3, b<<3);
}

// --------------------------------------------
//    update 32 bytes of the led matrix
//    ofs: which panel, 0 (ofs=0), 1 (ofs=32), 2 (ofs=64)...
// --------------------------------------------
void updatePixels(byte ofs, byte* buffer) {
  Serial.print("9");
  int currentLed = ofs*PIXELS_PER_PANEL;
  byte x=0;
  for (byte i=0; i < PIXELS_PER_PANEL; i++) {
    uint32_t color = convert15bitTo24bit(buffer[x]<<8 | buffer[x+1]);

    leds[currentLed].r = (color>>16)&255;
    leds[currentLed].b = (color>>8)&255; 
    leds[currentLed].g = color&255; 

    x+=2;
    currentLed++;
  }
  //FastSPI_LED.show();
  LEDS.show();
}

/* 
 --------------------------------------------
 read serial command
 --------------------------------------------
 read a string from the serial and store it in an array
 you must supply the str array variable
 returns number of bytes read, or zero if fail
 
 example ping command:
 		cmdfull[0] = START_OF_CMD (marker);
 		cmdfull[1] = addr;
 		cmdfull[2] = 0x01; 
 		cmdfull[3] = CMD_PING;
 		cmdfull[4] = START_OF_DATA (marker);
 		cmdfull[5] = 0x02;
 		cmdfull[6] = END_OF_DATA (marker);
 */


byte readCommand(byte *str) {
  byte b,i,sendlen;

  //wait until we get a CMD_START_BYTE or queue is empty
  i=0;
  while (Serial.available()>0 && i==0) {
    b = Serial.read();
    if (b == CMD_START_BYTE) {
      i=1;
    }
  }

  if (i==0) {
    //failed to get data ignore it
    g_errorCounter = 102;
    return 0;    
  }



  //read header  
  i=1;
  b=SERIAL_DELAY_LOOP;
  while (i<SERIAL_HEADER_SIZE) {
    if (Serial.available()) {
      str[i++] = Serial.read();
    } 
    else {
      delay(SERIAL_WAIT_DELAY); 
      if (b-- == 0) {
        g_errorCounter = 103;
        return 0;        //no data available!
      }      
    }
  }

  // --- START HEADER CHECK    
  //check if data is correct, 0x10 = START_OF_DATA
  if (str[4] != START_OF_DATA) {
    g_errorCounter = 104;
    return 0;
  }

  //check sendlen, its possible that sendlen is 0!
  sendlen = str[2];  
  // --- END HEADER CHECK

  //read data  
  i=0;
  b=SERIAL_DELAY_LOOP;
  while (i<sendlen+1) {
    if (Serial.available()) {
      str[SERIAL_HEADER_SIZE+i++] = Serial.read();
    } 
    else {
      delay(SERIAL_WAIT_DELAY); 
      if (b-- == 0) {
        g_errorCounter = 105;
        return 0;        //no data available!
      }      
    }
  }

  //check if data is correct, 0x20 = END_OF_DATA
  if (str[SERIAL_HEADER_SIZE+sendlen] != END_OF_DATA) {
    g_errorCounter = 106;
    return 0;
  }

  //return data size (without meta data)
  return sendlen;
}


/*
uint8_t readCommand(byte *str) {
  uint8_t recvNr = Serial.readBytes((char*)str, SERIAL_PACKET_SIZE);
  if (recvNr==0) {
    g_errorCounter = 102;
    return 0;        //no data available!    
  }

  //check header
  if (str[4] != START_OF_DATA) {
    g_errorCounter = 104;
    return 0;
  }
  
  uint8_t sendlen = str[2];  
  
  //check footer
  if (str[SERIAL_HEADER_SIZE+sendlen] != END_OF_DATA) {
    g_errorCounter = 106;
    return 0;
  }

  if (sendlen>recvNr) {
    g_errorCounter = 109;
    return 0;
  }

  return sendlen;
}
*/
