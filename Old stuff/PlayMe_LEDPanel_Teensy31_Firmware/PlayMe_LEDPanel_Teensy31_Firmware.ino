//

/*
 * PlayMe - LED Panel Teensy firmware
 * To be used either with Teensy 3.0 or 3.1 microcontrollers
 * 
 * ------------------------------------------------------------------------
 *
 * This firmware uses Hardware SPI. Unlike software SPI which is configurable,
 * hardware SPI works only on very specific pins. Refer to the Teensy PIN 
 * layout to check the details
 *
 * ------------------------------------------------------------------------
 *
 */

#include "FastLED.h"

//---- START USER CONFIG ----

#define DEBUG 1

//how many led pixels are connected
#define NUM_LEDS 128

//How many "universe" are connected, used to calculate offset
#define TOTAL_PACKET_SIZE 4

// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;

//---- END USER CONFIG ----

#define BAUD_RATE 115200

//define some tpm constants
#define TPM2NET_HEADER_SIZE 4
#define TPM2NET_HEADER_IDENT 0x9c
#define TPM2NET_CMD_DATAFRAME 0xda
#define TPM2NET_CMD_COMMAND 0xc0
#define TPM2NET_CMD_ANSWER 0xaa
#define TPM2NET_FOOTER_IDENT 0x36
#define SERIAL_FOOTER_SIZE 1

//3 byte per pixel or 24bit (RGB)
#define BPP 3

//package size we expect. 
#define MAX_PACKED_SIZE 520

#define PIXELS_PER_PACKET 170

// buffers for receiving and sending data
uint8_t packetBuffer[MAX_PACKED_SIZE]; //buffer to hold incoming packet
uint16_t psize;
uint8_t currentPacket;
uint8_t totalPacket;

CRGB leds[NUM_LEDS];

//********************************
// SETUP
//********************************
void setup() {  
  // Sanity check - allows reprogramming
  delay(2000);
  
  Serial.begin(BAUD_RATE);
  Serial.flush();
  Serial.setTimeout(20);
  #ifdef DEBUG  
    Serial.println("HI");
  #endif 

  pinMode(ledPin, OUTPUT);
  debugBlink(500);

  memset(packetBuffer, 0, MAX_PACKED_SIZE);


  FastLED.addLeds<WS2801, BRG>(leds, NUM_LEDS);
  
  //Alternative ways to define the LEDs
  //LEDS.addLeds<WS2801>(leds, NUM_LEDS);
  //LEDS.addLeds<WS2801, 11, 13, BRG, DATA_RATE_KHZ(400)>(leds, NUM_LEDS);
  //LEDS.addLeds<WS2811, 11, GRB>(leds, NUM_LEDS);  //connect on pin 11
    
  showInitImage();      // display some colors
}

//********************************
// LOOP
//********************************
void loop() {  
  int16_t res = readCommand();
  if (res > 0) {
  #ifdef DEBUG      
    Serial.print("FINE: ");
    Serial.print(psize, DEC);    
    Serial.print("/");
    Serial.print(currentPacket, DEC);    
    
    Serial.send_now();
  #endif
    digitalWrite(ledPin, HIGH);
    updatePixels();
    digitalWrite(ledPin, LOW);    
  }
  #ifdef DEBUG      
    else {
      if (res!=-1) {
        Serial.print("ERR: ");
        Serial.println(res, DEC);    
        Serial.send_now();
      }
    }
  #endif  
}

//********************************
// UPDATE PIXELS
//********************************
void updatePixels() {
  uint8_t nrOfPixels = psize/3;
  
  uint16_t ofs=0;
  uint16_t ledOffset = PIXELS_PER_PACKET*currentPacket;
  
  for (uint16_t i=0; i<nrOfPixels; i++) {
    leds[i+ledOffset] = CRGB(packetBuffer[ofs++], packetBuffer[ofs++], packetBuffer[ofs++]);    
  }
  
  //update only if all data packets recieved
  if (currentPacket==totalPacket-1) {
    #ifdef DEBUG      
      Serial.println("DRAW!");
      Serial.send_now();
    #endif    
    FastLED.show();
  } else {
    #ifdef DEBUG      
      Serial.print("NOTUPDATE: ");
      Serial.println(currentPacket, DEC);
      Serial.send_now();
    #endif        
  }
}

//********************************
// READ SERIAL PORT
//********************************
int16_t readCommand() {  
  uint8_t startChar = Serial.read();  
  if (startChar != TPM2NET_HEADER_IDENT) {
    return -1;
  }
  
  uint8_t dataFrame = Serial.read();
  if (dataFrame != TPM2NET_CMD_DATAFRAME) {
    return -2;  
  }
  
  uint8_t s1 = Serial.read();
  uint8_t s2 = Serial.read();  
  psize = (s1<<8) + s2;
  if (psize < 6 || psize > MAX_PACKED_SIZE) {
    return -3;
  }  

  currentPacket = Serial.read();  
  totalPacket = Serial.read();    
  if (totalPacket>TOTAL_PACKET_SIZE || currentPacket>TOTAL_PACKET_SIZE) {
    return -4;
  }
  
  //get remaining bytes
  uint16_t recvNr = Serial.readBytes((char *)packetBuffer, psize);
  if (recvNr!=psize) {
    return -5;
  }  

  uint8_t endChar = Serial.read();
  if (endChar != TPM2NET_FOOTER_IDENT) {
    return -6;
  }

  return psize;
}


// --------------------------------------------
//     create initial image
// --------------------------------------------
void showInitImage() {
  for (int i = 0 ; i < NUM_LEDS; i++ ) {
    //leds[i] = CRGB(i&255, (i>>1)&255, (i>>2)&255);
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void debugBlink(uint8_t t) {
  digitalWrite(ledPin, HIGH);
  delay(t);
  digitalWrite(ledPin, LOW);  
}



