/*
 * Strobot - LED Panel Teensy firmware
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

#define USE_USB_SERIAL         0
#define USE_UART_SERIAL        1

#define BAUD_RATE 115200               // USB Serial Baud rate - has no impact, Teensy always work at 12 Mbps

#define HWSERIAL               Serial1 // UART serial used to receive data forwarded by the RF microcontroller
#define HWSERIAL_BAUDRATE      6000000 // UART Serial baudrate - equal to 96MHz/16, absolute maximum available with Teensy 3.2
#define RX_COM_REINIT_TIMEOUT  30000   // If no frame is received after 20 seconds, consider the RF communication link to be down

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
#define MAX_PACKET_SIZE 520

#define PIXELS_PER_PACKET 170

// buffers for receiving and sending data
uint8_t packetBuffer[MAX_PACKET_SIZE]; //buffer to hold incoming packet
uint16_t psize;
uint8_t currentPacket;
uint8_t totalPacket;

uint8_t serialMode;

unsigned long communicationTimeout_timer;

CRGB leds[NUM_LEDS];



//********************************
// UPDATE PIXELS
//********************************
void updatePixels() {
  uint8_t nrOfPixels = psize/3;
  
  uint16_t ofs=0;
  uint16_t ledOffset = PIXELS_PER_PACKET*currentPacket;

  //Serial.print("Array:");
  for (uint16_t i=0; i<nrOfPixels; i++) {
//    Serial.print("[");
//    Serial.print(packetBuffer[ofs]);
//    Serial.print(",");
//    Serial.print(packetBuffer[ofs+1]);
//    Serial.print(",");
//    Serial.print(packetBuffer[ofs+2]);
//    Serial.print("] / ");
    leds[i+ledOffset] = CRGB(packetBuffer[ofs++], packetBuffer[ofs++], packetBuffer[ofs++]);    
  }
  //Serial.println("///");
  
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
// READ SERIAL PORT - USB Serial
//********************************
int16_t readCommand_usbSerial() {
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
  if (psize < 6 || psize > MAX_PACKET_SIZE) {
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

  communicationTimeout_timer = millis();
  
  return psize;
}

//********************************
// READ SERIAL PORT - UART Serial
//********************************
int16_t readCommand_uartSerial() {
  
  if (HWSERIAL.available() > 6) {
    uint8_t startChar = HWSERIAL.read();  
    if (startChar != TPM2NET_HEADER_IDENT) {
      #ifdef DEBUG
        Serial.print("NotStartChar");
        Serial.println(startChar,DEC);
      #endif
      return -1;
    }
    
    uint8_t dataFrame = HWSERIAL.read();
    if (dataFrame != TPM2NET_CMD_DATAFRAME) {
      #ifdef DEBUG
        Serial.print("DataFrame err. dataFrame:");
        Serial.print(dataFrame,DEC);
        Serial.print(". Expected TPM2NET_CMD_DATAFRAME:");
        Serial.println(TPM2NET_CMD_DATAFRAME,DEC);
      #endif
      return -2;  
    }
    
    uint8_t s1 = HWSERIAL.read();
    uint8_t s2 = HWSERIAL.read();  
    psize = (s1<<8) + s2;
    if (psize < 6 || psize > MAX_PACKET_SIZE) {
      #ifdef DEBUG
        Serial.print("Size err. s1:");
        Serial.print(s1,DEC);
        Serial.print(". s2:");
        Serial.print(s2,DEC);
        Serial.print(". psize:");
        Serial.println(psize,DEC);
      #endif
      return -3;
    }  
  
    currentPacket = HWSERIAL.read();  
    totalPacket = HWSERIAL.read();    
    if (totalPacket>TOTAL_PACKET_SIZE || currentPacket>TOTAL_PACKET_SIZE) {
      #ifdef DEBUG
        Serial.print("Packet# err. currentPacket:");
        Serial.print(currentPacket,DEC);
        Serial.print(". totalPacket:");
        Serial.print(totalPacket,DEC);
        
        Serial.print(". startChar:");
        Serial.print(startChar,DEC);
        Serial.print(". dataFrame:");
        Serial.print(dataFrame,DEC);
        Serial.print(". psize:");
        Serial.print(psize,DEC);
        
        Serial.print(". Expected TOTAL_PACKET_SIZE:");
        Serial.println(TOTAL_PACKET_SIZE,DEC);
        
        Serial.print(HWSERIAL.read(),DEC);
        Serial.print("/");
        Serial.print(HWSERIAL.read(),DEC);
        Serial.print("/");
        Serial.print(HWSERIAL.read(),DEC);
        Serial.print("/");
        Serial.println(HWSERIAL.read(),DEC);
      #endif
      return -4;
    }
    
    //get remaining bytes
    uint16_t recvNr = HWSERIAL.readBytes((char *)packetBuffer, psize);
    if (recvNr!=psize) {
      return -5;
    }  

    //Wait until the data is correctly transmitted over the serial line
    long rxTimer = millis();
    while (HWSERIAL.available() <= 0 && (millis()-rxTimer < 3) ) {
      // Do nothing
    }
  
    uint8_t endChar = HWSERIAL.read();
    if (endChar != TPM2NET_FOOTER_IDENT) {
      #ifdef DEBUG
        Serial.print("End char err. end Char:");
        Serial.print(endChar,DEC);
        Serial.print(". TPM2NET_FOOTER_IDENT:");
        Serial.print(TPM2NET_FOOTER_IDENT,DEC);
        
        Serial.print(". startChar:");
        Serial.print(startChar,DEC);
        Serial.print(". dataFrame:");
        Serial.print(dataFrame,DEC);
        Serial.print(". psize:");
        Serial.print(psize,DEC);
        
        Serial.print(". Expected TOTAL_PACKET_SIZE:");
        Serial.println(TOTAL_PACKET_SIZE,DEC);

      #endif
      return -6;
    }
      
    return psize;
  }
  else {
    return -1;
  }
  
}


// --------------------------------------------
//     create initial image
// --------------------------------------------
void showInitImage() {
  for (int i = 0 ; i < NUM_LEDS; i++ ) {
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void debugBlink(uint16_t t) {
  digitalWrite(ledPin, HIGH);
  delay(t);
  digitalWrite(ledPin, LOW);  
}



//********************************
// SETUP
//********************************
void setup() {
  // Sanity check - allows reprogramming
  delay(2000);
  
  Serial.begin(BAUD_RATE);                 // Initialize the first serial link (USB)
  Serial.flush();
  Serial.setTimeout(20);

  HWSERIAL.begin(HWSERIAL_BAUDRATE);       // Initialize the second Serial link (RF microcontroller)
  HWSERIAL.flush();
  HWSERIAL.setTimeout(30);
    
  #ifdef DEBUG  
    Serial.println("HI");
  #endif 

  serialMode = USE_UART_SERIAL;            // Default mode: try to use the RF24 communication link
  
  pinMode(ledPin, OUTPUT);
  debugBlink(500);

  memset(packetBuffer, 0, MAX_PACKET_SIZE);
  
  FastLED.addLeds<WS2801, BRG>(leds, NUM_LEDS);
  
  showInitImage();      // display some colors

  communicationTimeout_timer = millis();   // This timer contains the timestamp of the last frame received through the USB serial link
}

//********************************
// LOOP
//********************************
void loop() {
  int16_t res;
  if (serialMode == USE_UART_SERIAL) {
    if (Serial.available()) {
      res = readCommand_usbSerial();
      if (res > 0) {
        serialMode = USE_USB_SERIAL;
      }
    }
  }

  if (serialMode == USE_USB_SERIAL && millis() - communicationTimeout_timer > RX_COM_REINIT_TIMEOUT) {
    serialMode = USE_UART_SERIAL;
  }
    
  
  if (serialMode == USE_USB_SERIAL) {
    res = readCommand_usbSerial();
  }
  else {
    res = readCommand_uartSerial();
  }

  
  if (res > 0) {
  #ifdef DEBUG      
    Serial.print("FINE: ");
    Serial.print(psize, DEC);    
    Serial.print("/");
    Serial.println(currentPacket, DEC);    
    
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


