/*
** Copyright Martin Di Rollo / XI - 2016
**
** //// RF24 emitter/receiver /////
** This program aims to either to wirelessly transmit data stemming from Strobot,
** or to receive wireless data over RF24 in order to display images on LED panels
* 
*  Each image to transmit is segmented into multiple frames
*  As each packet sent by NRF24 is 32 bytes long, the idea is to send
*  multiple 24 bytes long frames, each including a header of which LEDs
*  the current frame refers to.
*  With regular RGB LEDs, 3 bytes are required per pixel. A 24 byte long
*  frame will cover 8 LEDs.
*  In the case of an 8*16 LED array, this means that 16 frames are needed
*  to cover a single image
*  
*  Upon receiving (using the serial link) a new image, the firmware breaks
*  down this image into individual data.
*  Then, a loop sends indefinitely all the separate frames until a new
*  image is received.
*  An important number of redundant frames will be sent for each image, this
*  is intentional by design, in order to avoid critical frame drops.
*  The downside is that, without proper implementation on the RX side,
*  the LEDs will be refreshed by lines of 8 LEDs.
* 
*  On the other side of the loop, the individual packets are received and
*  consolidated as a full frame, and sent to the LED microcontroller as a
*  regular TPM2 frame - the second microcontroller expects the same frames
*  coming from either the RF microcontroller, or from a USB connection
*  through the PC (directly controlled by Strobot).
**
*/

#include <SPI.h>
#include "RF24.h"

#define DEBUG                  1

/*************  Hardware Definitions *******************************/
#define ID                     0        // Define the ID of the panel to communicate with, range is [0,4]
/*
#define ID                     1
#define ID                     2
#define ID                     3
#define ID                     4
*/

//#define ROLE_TX                0       // Define the role of this unit: TX for Strobot side, RX for panel side
#define ROLE_RX                1

/*************  Hardware Definitions - General  *****************/
#define NB_LEDS                128
#define BYTES_PER_PIXEL        3
#define USEFUL_PAYLOAD_SIZE    9    // Each packet contains the data for a few leds LEDs, and a header      
#define RF_FRAME_SIZE          10    // 32 is the hardware limitation of RF24 modules, but data transmission is more reliable with smaller packets
#define HEADER_SIZE            RF_FRAME_SIZE - USEFUL_PAYLOAD_SIZE
#define NB_FRAMES              int(ceil(NB_LEDS*BYTES_PER_PIXEL/(USEFUL_PAYLOAD_SIZE)))
#define NB_PIXELS_PER_FRAME    int(USEFUL_PAYLOAD_SIZE/BYTES_PER_PIXEL)
//#define FRAMESIZE_IN_HEADER    1
#define SHORT_RF_ADDRESS       1       // Use 24 bit addresses instead of 40 bit ones
#define ENABLE_CRC             1

/*************  Hardware Definitions - TX side only  ************/
#define TX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

/*************  Hardware Definitions - RX side only  ************/
#define NB_FRAMES_TX_PER_SEC   60      // Only used on the RX side: number of frames the receiver shall send to the LED microcontroller per second
#define RX_FRAME_TX_PERIOD_MS  int(1000/NB_FRAMES_TX_PER_SEC)
#define HWSERIAL               Serial1 // UART serial used to forward data to the LED microcontroller
#define HWSERIAL_BAUDRATE      6000000 // UART Serial baudrate - equal to 96MHz/16, absolute maximum available with Teensy 3.2
#define RX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

/*************  TPM2 Definitions *******************************/
#define TPM2NET_HEADER_SIZE 4
#define TPM2NET_HEADER_IDENT 0x9c
#define TPM2NET_CMD_DATAFRAME 0xda
#define TPM2NET_CMD_COMMAND 0xc0
#define TPM2NET_CMD_ANSWER 0xaa
#define TPM2NET_FOOTER_IDENT 0x36
#define SERIAL_FOOTER_SIZE 1

//package size we expect
#define MAX_PACKET_SIZE 520
#define PIXELS_PER_PACKET 170

//How many "universe" are connected, used to calculate offset
#define TOTAL_PACKET_SIZE 4

/*************  Data Buffers for incoming serial packets *******/
uint8_t packetBuffer[MAX_PACKET_SIZE];
uint16_t psize;
uint8_t currentPacket;
uint8_t totalPacket;
uint8_t packetNb = 0;                   // Current packet to be sent (subset of LED sent)

/*************  RX side data information  *********************/
uint8_t frameSize_msb;
uint8_t frameSize_lsb;
uint16_t frameSize = 0x0000;
const uint8_t rx_packetnumber = 0;      // Todo: implement multi-packet transmission
const uint8_t rx_totalpackets = 1;
const uint8_t predefined_frameSize_msb = (uint8_t) ((NB_LEDS * BYTES_PER_PIXEL) >> 8);
const uint8_t predefined_frameSize_lsb = (uint8_t) ((NB_LEDS * BYTES_PER_PIXEL) & 0xFF);

/*************  Hardware RF Configuration  *********************/
RF24 radio(7,8);                        // Set up nRF24L01 radio on SPI bus plus pins 7 & 8
const int ledPin = 13;                  // Teensy 3.0 has the LED on pin 13

/***************************************************************/

// Radio pipe addresses for the nodes to communicate.
// The five first addresses are the panels', the following are the local emitters' addresses
// These uint64_t addresses are used in the 40-bit address mode. To optimize the system, shave off two bytes and use 24 bit long addresses
const uint64_t pipes[10] = { 0xABCDABCD71LL, 0x544d52687CLL, 
                             0x544d526832LL, 0x544d52683CLL,
                             0x544d526846LL, 0x544d526850LL,
                             0x544d52685ALL, 0x544d526820LL, 
                             0x544d52686ELL, 0x544d52684BLL};
const uint32_t short_pipes[10] = { 0xABCD71L, 0x544d7CL, 
                                   0x544D32L, 0x544D3CL,
                                   0x544D46L, 0x544D50L,
                                   0x544D5AL, 0x544D20L, 
                                   0x544D6EL, 0x544D4BL};

uint8_t tx_data[NB_FRAMES][RF_FRAME_SIZE];   // Data buffers to send using the NRF24 antenna
uint8_t temp_rx_data[RF_FRAME_SIZE];         // Temp buffer to receive data before checking the contents

bool communication_init = false;             // Flag set to true when the communication link is initialized

unsigned long syncTime;
unsigned long rxTimer;
unsigned long txTimer;
unsigned long tx_last_received_frame_timestamp;
unsigned long rx_last_received_frame_timestamp;

void setup(void) {

  // Sanity check - allows reprogramming
  delay(2000);

  Serial.begin(115200);                      // With Teensy microcontrollers, serial links always work at 12Mbps
  Serial.flush();
  Serial.setTimeout(10);                     // Use a very short Serial read timeout for the USB serial, as we need to do other stuff
  #ifdef DEBUG  
    Serial.println("HI");
  #endif

  #ifdef ROLE_RX
    HWSERIAL.begin(HWSERIAL_BAUDRATE);                  // Initialize the second Serial link
    HWSERIAL.flush();
    HWSERIAL.setTimeout(20);
  #endif

  memset(packetBuffer, 0, MAX_PACKET_SIZE);  // Initialize the buffer to hold incoming packet data

  radio.begin();                             // Setup and configure rf radio
  radio.setChannel(ID);                      // Set the RF channel to be equal to the ID (range [0,127])
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  //radio.enableDynamicAck();                  // This MUST be called prior to attempting single write NOACK calls
  radio.setAutoAck(0);                       // Ensure autoACK is disabled - we don't need to know whether the packets are actually received
  //radio.setRetries(2,15);                    // Optionally, increase the delay between retries & # of retries

  #ifdef ENABLE_CRC
    radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance
  #else
    radio.disableCRC();
  #endif


  #ifdef SHORT_RF_ADDRESS
    #ifdef ROLE_RX
      radio.setAddressWidth(3);
      radio.openWritingPipe(short_pipes[ID]);
      radio.openReadingPipe(1,short_pipes[ID+5]);
      radio.startListening();                  // Start listening
    #endif
    #ifdef ROLE_TX
      radio.setAddressWidth(3);
      radio.openWritingPipe(short_pipes[ID+5]);
      radio.openReadingPipe(1,short_pipes[ID]);
      radio.stopListening();                   // Do not listen
    #endif
  #else
    #ifdef ROLE_RX
      radio.openWritingPipe(pipes[ID]);
      radio.openReadingPipe(1,pipes[ID+5]);
      radio.startListening();                  // Start listening
    #endif
    #ifdef ROLE_TX
      radio.openWritingPipe(pipes[ID+5]);
      radio.openReadingPipe(1,pipes[ID]);
      radio.stopListening();                   // Do not listen
    #endif
  #endif

  #ifdef DEBUG
    radio.printDetails();                    // Dump the configuration of the rf unit for debugging
  #endif
  #ifdef ROLE_TX
    Serial.print(F("\n\rXI LED Panel - USB Serial to RF24 - ID "));
    Serial.println(ID);
  #endif
  #ifdef ROLE_RX
    Serial.print(F("\n\rXI LED Panel - RF24 RX to LED Panel - ID "));
    Serial.println(ID);
  #endif

  // Initialize the TX data buffers
  for (int i=0; i<NB_FRAMES; i++) {
    memset(tx_data[i], 0, RF_FRAME_SIZE);    // Fill all RF_FRAME_SIZE bytes of tx_data[i] with 0's
    tx_data[i][0] = i*NB_PIXELS_PER_FRAME;   // And put in the the LED offset
  }
  memset(temp_rx_data, 0, RF_FRAME_SIZE);    // Do the same for the temp RX buffer
  
  
  radio.powerUp();                           // Power up the radio

  syncTime = micros();                       // Initialize the PLL sync time
  rxTimer  = millis();                       // Initialize the RX side frame transmission timer
  rx_last_received_frame_timestamp = millis();
  tx_last_received_frame_timestamp = millis();
  
}

void loop(void){

  /*************  TX *******************/
  #ifdef ROLE_TX
    if (Serial.available() > 0) {
      int16_t res = readCommand();             // Check if Strobot has sent new data
      if (res > 0) {
        update_tx_data_buffers();              // Fill in the TX data buffers - if no seqNum is included in the frame header, the update can be done here
        //send_all_packets();                    // And now send all packets
        #ifdef DEBUG
          Serial.println("FINE");
          //Serial.print(psize, DEC);    
          //Serial.print("/");
          //Serial.print(currentPacket, DEC);    
          //Serial.send_now();
        #endif

      }
      #ifdef DEBUG   
        else {
          if (res!=-1) {                       // A problem has occured upon receiving the latest packet
            Serial.print("ERR: ");             // This can happen if too much data is sent on the serial link
            Serial.println(res, DEC);          // or if the microcontroller is not fast enough (ie. standard
            Serial.send_now();                 // Arduino instead of the faster Teensy 3.x modules)
          }
        }
      #endif
    }

    
    if (communication_init) {
      //update_tx_data_buffers();              // Fill in the TX data buffers - if a seqNum is included in the frame, the update must be done here

      //send_all_packets();                    // And now send all packets
      send_specific_packet();                  // Send one specific packet - sending all of them would freeze the Teensy, while new frames are still being received

      // Actually not that good of an idea regarding TX : always try to send data, you never know
      //if (millis() - tx_last_received_frame_timestamp > TX_COM_REINIT_TIMEOUT) {
      //  communication_init = false;
      //}
    }
    
  #endif
  
  
  /*************  RX *******************/

  #ifdef ROLE_RX
    while(radio.available()){
     long readTime = micros();
     radio.read(&temp_rx_data,RF_FRAME_SIZE);
     Serial.println(micros() - readTime);
     communication_init = true;
     
     // Check the contents of the received data and put it at the appropriate offset in the frame buffer
     process_rx_data();
     
     #ifdef DEBUG
       Serial.print("RX: ");
       Serial.print(temp_rx_data[0], DEC);
       Serial.print(" / ");
       Serial.print(temp_rx_data[0 + HEADER_SIZE], DEC);
       Serial.print(" / ");
       Serial.print(temp_rx_data[1 + HEADER_SIZE], DEC);
       Serial.print(" / ");
       Serial.println(temp_rx_data[2 + HEADER_SIZE], DEC);
     #endif
     rx_last_received_frame_timestamp = millis();
    }

    // Send a consolidated TPM2 frame to the LED microcontroller every RX_FRAME_TX_PERIOD_MS milliseconds
    if(millis() - rxTimer > RX_FRAME_TX_PERIOD_MS && communication_init) {
      rxTimer = millis();
      transmit_consolidated_frame();
      #ifdef DEBUG
        Serial.print("-> TX:");
        Serial.println(packetBuffer[0]);
      #endif
    }

    if (millis() - rx_last_received_frame_timestamp > RX_COM_REINIT_TIMEOUT) {
      communication_init = false;
    }
  #endif
  
}


/*************  Read the serial port *******************/
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
  if (psize < 6 || psize > MAX_PACKET_SIZE) {
    return -3;
  }
  #ifdef FRAMESIZE_IN_HEADER
    if (!communication_init) {
      for (int i=0; i<NB_FRAMES; i++) {
        tx_data[i][1] = s1;                    // Put in the header of the frames to send the MSB of the frame's size
        tx_data[i][2] = s2;                    // Put in the header of the frames to send the LSB of the frame's size
      }
    }
  #endif

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

  communication_init = true;
  tx_last_received_frame_timestamp = millis();
  
  return psize;
}

/*************  Update the data to send *******************/
void update_tx_data_buffers() {
  for (int i=0; i<NB_FRAMES; i++) {
    memcpy(tx_data[i] + HEADER_SIZE, packetBuffer + i*USEFUL_PAYLOAD_SIZE, USEFUL_PAYLOAD_SIZE);
  }
}

void process_rx_data() {
  memcpy(packetBuffer + temp_rx_data[0]*BYTES_PER_PIXEL, temp_rx_data + HEADER_SIZE, USEFUL_PAYLOAD_SIZE);
  #ifdef FRAMESIZE_IN_HEADER
    frameSize_msb = temp_rx_data[1];      // Using smaller packets, no overhead left to send the frame side. And anyways, the size is always the same, set to BYTES_PER_PIXEL * NB_LEDS
    frameSize_lsb = temp_rx_data[2];
    frameSize     = (uint16_t) frameSize_msb << 8;
    frameSize |= frameSize_lsb;           // Note: this frame size is the total frame size, not the size of this particular packet
  #endif
}

void send_all_packets() {
  #ifdef DEBUG
    Serial.print("RF-TX:");
    txTimer = micros();
  #endif
  for (int i=0; i<NB_FRAMES; i++) {
    // Redundancy trick: when packet loss occurs, it is generally almost at the same time.
    // Rather than sending twice the same package, send the beginning and the end of the frame
    // --- Not possible with this hardware, we do not have enough time to send all packets twice with low-payload packets
    radio.writeFast(&tx_data[i], RF_FRAME_SIZE, 1);    // Do not ask for an ACK when writing this packet
    
    //radio.writeFast(&tx_data[NB_FRAMES-1-i], RF_FRAME_SIZE, 1);
    
    #ifdef DEBUG
      //Serial.print(i,DEC);
      //Serial.print("/");
      //Serial.print(tx_data[i][0]);
    #endif

    if(micros() - syncTime > 3500) {          // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = micros();                 // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      radio.txStandBy();         // This gives the PLL time to sync back up
      delayMicroseconds(130);
      #ifdef DEBUG
        Serial.print("/");
        Serial.send_now();
      #endif
    }
  }

  //bool ok = radio.txStandBy();
  
  #ifdef DEBUG
    Serial.print("-->over ");
    Serial.println(micros() - txTimer, DEC);
  #endif
  
}

void send_specific_packet() {
  radio.writeFast(&tx_data[packetNb], RF_FRAME_SIZE, 1);    // Do not ask for an ACK when writing this packet
  #ifdef DEBUG
    if (packetNb == 0) {
      Serial.print("RF-TX:");
      txTimer = micros();
    }
    else if (packetNb == NB_FRAMES - 1) {
      Serial.print("-->over ");
      Serial.println(micros() - txTimer, DEC);
    }
  #endif
  packetNb = (packetNb+1)%NB_FRAMES;                        // Increment for the next loop
  
  if(micros() - syncTime > 3500) {          // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = millis();               // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      radio.txStandBy();                // This gives the PLL time to sync back up
      //delayMicroseconds(130);
      #ifdef DEBUG
        Serial.print("/");
        Serial.send_now();
      #endif
    }
}

void transmit_consolidated_frame() {
  long microTimer = micros();
  HWSERIAL.write(TPM2NET_HEADER_IDENT);
  HWSERIAL.write(TPM2NET_CMD_DATAFRAME);
  //HWSERIAL.write(frameSize_msb);
  //HWSERIAL.write(frameSize_lsb);
  HWSERIAL.write(predefined_frameSize_msb);  // Frame size MSB (1)   - not enough space left in the header to include the frame size
  HWSERIAL.write(predefined_frameSize_lsb);  // Frame size LSB (128) - anyways, the size is always the same, as it is hardware-dependant
  
  //HWSERIAL.flush();
  HWSERIAL.write(rx_packetnumber);           // Todo: implement multi-packet transmission, by adding the info in the header
  HWSERIAL.write(rx_totalpackets);           // of the frames sent by the Strobot multicontroller
  //HWSERIAL.flush();
  //for (int i=0; i<frameSize; i++) {          // Write the contents of the buffer
  //  HWSERIAL.write(packetBuffer[i]);
  //}
  //HWSERIAL.write(packetBuffer, frameSize);   // Write the contents of the buffer
  HWSERIAL.write(packetBuffer, (NB_LEDS * BYTES_PER_PIXEL));   // Write the contents of the buffer
  
  HWSERIAL.write(TPM2NET_FOOTER_IDENT);
  //HWSERIAL.flush();

  #ifdef DEBUG
    Serial.print("TXLED::");
    //Serial.print(TPM2NET_HEADER_IDENT, DEC);
    //Serial.print(". CMD:");
    //Serial.print(TPM2NET_CMD_DATAFRAME, DEC);
    //Serial.print(". Size MSB:");
    //Serial.print(frameSize_msb, DEC);
    //Serial.print(". Size LSB:");
    //Serial.print(frameSize_lsb, DEC);
    //Serial.print(". Size:");
    //Serial.print((frameSize_msb<<8) + frameSize_lsb, DEC);
    //Serial.print(". RealSize:");
    //Serial.print(frameSize, DEC);
    //Serial.print(". rx_packetnumber:");
    //Serial.print(rx_packetnumber, DEC);
    //Serial.print(". rx_totalpackets:");
    //Serial.print(rx_totalpackets, DEC);
    //Serial.print(". packetBuffer[0]:");
    //Serial.print(packetBuffer[0], DEC);
    //Serial.print(". packetBuffer[1]:");
    //Serial.print(packetBuffer[1], DEC);
    Serial.print(". Time:");
    Serial.print(micros()-microTimer,DEC);
  #endif
  
}

