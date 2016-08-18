/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define NUMBER_OF_DEVICES 7
#define DEVICE_NUMBER 0
#define RF_CHANNEL 96
#define DEBUG 1


//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);

// Variable to store the last incoming animation
int currentAnimation = 0;


//
// Topology
//

//// Radio pipe addresses for the 2 nodes to communicate.
//const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// Topology : Radio pipe addresses for 7 nodes to communicate, plus one for the master channel
const uint64_t masterPipe = 0xF0F0F0F011LL;

const uint64_t pipes[NUMBER_OF_DEVICES] = { 0xF0F0F0F022LL, 
                                            0xF0F0F0F033LL, 
                                            0xF0F0F0F044LL, 
                                            0xF0F0F0F055LL, 
                                            0xF0F0F0F066LL, 
                                            0xF0F0F0F077LL,
                                            0xF0F0F0F088LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the role_pin
//

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

// The role of the current running sketch
role_e role;

void setup(void)
{

  // read the address pin, establish our role
  role = role_pong_back;

  //
  // Print preamble
  //

  Serial.begin(57600);
  printf_begin();
  //printf("\n\rRF24/examples/pingpair/\n\r");
  //printf("ROLE: %s\n\r",role_friendly_name[role]);

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(0,0);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)


  radio.openWritingPipe(masterPipe);
  radio.openReadingPipe(1,pipes[DEVICE_NUMBER]);

  //
  // Start listening to catch any incoming packet on this pipe
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  //radio.printDetails();
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
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &incoming_val, sizeof(int) );

        if (currentAnimation != incoming_val) {
          currentAnimation = incoming_val;
          Serial.print(incoming_val);
          Serial.print("w");
        }
      }

      // First, stop listening so we can talk
      //radio.stopListening();

      // Send the final one back.
      //radio.write( &incoming_val, sizeof(int) );
      //printf("Sent response.\n\r");


    }
}

