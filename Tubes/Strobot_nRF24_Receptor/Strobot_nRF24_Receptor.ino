/*
** Copyright Martin Di Rollo / PlayMe - 2014
**
** //// nRF24 receptor /////
** This program aims to wirelessly receive commands from the master PlayMeLightSetup
** The microcontroller running this sketch is interfaced with another uC, and
** sends the commands it receives using the Serial link
** Important : no print is to be made except from the necessary commands, or else it
** would mess with the data interpretation of the distant microcontroller
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define NUMBER_OF_DEVICES 4
#define DEVICE_NUMBER 0
#define RF_CHANNEL 96
#define DEBUG 1

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Variable to store the last incoming animation
int currentAnimation = 0;

// Topology : Radio pipe addresses for 4 nodes to communicate, plus one for the master channel
const uint64_t masterPipe = 0xF0F0F0F011LL;
const uint64_t pipes[NUMBER_OF_DEVICES] = { 0xF0F0F0F022LL, 
                                            0xF0F0F0F033LL, 
                                            0xF0F0F0F044LL, 
                                            0xF0F0F0F055LL};


void setup(void)
{

  Serial.begin(57600);
  printf_begin();

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

  // Open pipes to other nodes for communication
  radio.openWritingPipe(masterPipe);
  radio.openReadingPipe(1,pipes[DEVICE_NUMBER]);

  // Start listening to catch any incoming packet on this pipe
  radio.startListening();

  // Dump the configuration of the rf unit (not to be uncommented, this messes up the program : 
  // the data is sent over the Tx link, which goes in turn to the uC paired up to this receptor)
  // radio.printDetails();
}

void loop(void)
{ 
    
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      int incoming_val;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and check if this was the last one.
        done = radio.read( &incoming_val, sizeof(int) );
        if (currentAnimation != incoming_val) {
          currentAnimation = incoming_val;
          Serial.print(currentAnimation);
        }
      }
    }
}

