/*
** Copyright Martin Di Rollo / PlayMe - 2014
**
** //// USB to RF24 /////
** This program aims to wirelessly transmit commands from the master PlayMeLightSetup
** Processing sketch in order to control various custom devices
** 
** The animations for custom devices are set using these commands:
**
** <number>c : Select device (channel)
** <number>v : Set animation to new value
**
** These can be combined. For example:
** 100c355w : Set channel 100 to value 255.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define NUMBER_OF_DEVICES 4
#define RF_CHANNEL 96
#define DEBUG 1


// Hardware configuration
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);


// Topology
// Radio pipe addresses for 4 nodes to communicate, plus one for the master channel
const uint64_t masterPipe = 0xF0F0F0F011LL;

const uint64_t pipes[NUMBER_OF_DEVICES] = { 0xF0F0F0F022LL, 
                                            0xF0F0F0F033LL, 
                                            0xF0F0F0F044LL, 
                                            0xF0F0F0F055LL };

// Table containing the animations to be played, for devices whose number range from 0 to 3
int currentAnimations[NUMBER_OF_DEVICES] = {0,0,0,0};

//Temporary variables used to receive data from Processing
int value = 0;
int channel = 0;

void setup(void) {

  Serial.begin(57600);
  printf_begin();

  printf("\n\rUSB Serial to nRF24 \n\r");
  printf("SerialToCustomDevice ready\n\r");
  printf("\n\r");
  printf("Syntax to use with Processing sketch:\n\r");
  printf("  123c : use device # 123\n");
  printf("  45w  : set current device to animation 45\n\r");
  printf("\n\r");


  // Setup and configure rf radio
  radio.begin();

  // Set the delay between retries & the number of retries to 0
  radio.setRetries(0,0);

  // Reduce the payload size to improve reliability
  radio.setPayloadSize(8);
  
  //Set the Power Amplifier level to the max level
  radio.setPALevel(RF24_PA_MAX);

  // Set the data rate to 250 kbps, the minimum available - this allows for better range
  radio.setDataRate(RF24_250KBPS);



  // Stop listening : this sketch only aims to push data as fast as possible
  radio.stopListening();
  
  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

void loop(void)
{
    for (int i=0; i<NUMBER_OF_DEVICES; i++) {
      //If serial data is available, this loop will be preempted
      //Go directly to the serial processing step
      if (Serial.available()) {
        break;
      }
      else {
        setAnimation(i);
      }
    }
  
    // Check if data coming from Processing is available
    int c;
    if (Serial.available()) {
      c = Serial.read();
      if ((c>='0') && (c<='9')) {
        value = 10*value + c - '0';
      } else {
        if (c=='c') channel = value;
        else if (c=='w') {
          currentAnimations[channel] = value;
          //Save a slight bit of time by sending the command for this channel right away
          //This allows the system to be slightly more reactive
          setAnimation(channel);  
          //#ifdef DEBUG
            //printf("Received command for channel %d : animation %d\n", channel, value);
            printf("Current animation table : %d, %d, %d, %d\n", currentAnimations[0], currentAnimations[1],currentAnimations[2],currentAnimations[3]);
          //#endif
        }
        value = 0;
      }
    }  
}

void setAnimation(int deviceNumber) {
  radio.openWritingPipe(pipes[deviceNumber]);
  radio.write( &currentAnimations[deviceNumber], sizeof(int) );
}
