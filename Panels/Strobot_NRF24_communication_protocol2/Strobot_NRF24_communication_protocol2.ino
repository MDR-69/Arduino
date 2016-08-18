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

#define ROLE_TX                0       // Define the role of this unit: TX for Strobot side, RX for panel side
//#define ROLE_RX                1

/*************  Hardware Definitions - General  *****************/
#define NB_LEDS                128
#define BYTES_PER_PIXEL        3
#define USEFUL_PAYLOAD_SIZE    24
#define RF_FRAME_SIZE          32      // Hardware limitation of RF24 modules
#define HEADER_SIZE            RF_FRAME_SIZE - USEFUL_PAYLOAD_SIZE
#define NB_FRAMES              int(ceil(NB_LEDS*BYTES_PER_PIXEL/(USEFUL_PAYLOAD_SIZE)))
#define NB_PIXELS_PER_FRAME    int(USEFUL_PAYLOAD_SIZE/BYTES_PER_PIXEL)

/*************  Hardware Definitions - TX side only  ************/
#define TX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

/*************  Hardware Definitions - RX side only  ************/
#define MIN_NBFRAMES_TX_PERSEC 25      // Only used on the RX side: minimum number of frames the receiver shall send to the LED microcontroller per second
#define RX_FRAME_TX_PERIOD_MS  int(1000/MIN_NBFRAMES_TX_PERSEC)
#define HWSERIAL               Serial1 // UART serial used to forward data to the LED microcontroller
#define HWSERIAL_BAUDRATE      640000
#define RX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

#define RX_QUEUE_DEPTH         10      // Number of frames in the current frame buffer

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
uint8_t packetBuffer[MAX_PACKET_SIZE];  // Only used to the TX side
uint16_t psize;
uint8_t currentPacket;
uint8_t totalPacket;
uint8_t seqNum = 0;                     // ID referring to the current frame concerned by the sub packet, sent in the TX header

/*************  RX side data information  *********************/
uint8_t frameSize_msb;
uint8_t frameSize_lsb;
uint16_t frameSize = 0x0000;
const uint8_t rx_packetnumber = 1;      // Todo: implement multi-packet transmission
const uint8_t rx_totalpackets = 1;
uint8_t rx_packetBuffer[RX_QUEUE_DEPTH][MAX_PACKET_SIZE];
bool    rx_seqNum_table[RX_QUEUE_DEPTH][NB_FRAMES];         // Used to store the currently received packet sequence numbers
uint8_t rx_currentSeqNum;               // Store the sequence number of the last frame sent to the LED microcontroller

/*************  Hardware RF Configuration  *********************/
RF24 radio(7,8);                        // Set up nRF24L01 radio on SPI bus plus pins 7 & 8
const int ledPin = 13;                  // Teensy 3.0 has the LED on pin 13

/***************************************************************/

// Radio pipe addresses for the nodes to communicate.
// The five first addresses are the panels', the following are the local emitters' addresses
const uint64_t pipes[10] = { 0xABCDABCD71LL, 0x544d52687CLL, 
                             0x544d526832LL, 0x544d52683CLL,
                             0x544d526846LL, 0x544d526850LL,
                             0x544d52685ALL, 0x544d526820LL, 
                             0x544d52686ELL, 0x544d52684BLL};

uint8_t tx_data[NB_FRAMES][32];              // Data buffers to send using the NRF24 antenna
uint8_t temp_rx_data[32];                    // Temp buffer to receive data before checking the contents

bool communication_init = false;             // Flag set to true when the communication link is initialized

unsigned long syncTime;
unsigned long rxTimer;
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
    HWSERIAL.begin(HWSERIAL_BAUDRATE);       // Initialize the second Serial link
    HWSERIAL.flush();
    HWSERIAL.setTimeout(20);
  #endif

  memset(packetBuffer, 0, MAX_PACKET_SIZE);  // Initialize the buffer to hold incoming packet data
  for (int i=0;i<RX_QUEUE_DEPTH;i++) {
    memset(rx_packetBuffer[i], 0, MAX_PACKET_SIZE);
  }
  for (int i=0;i<RX_QUEUE_DEPTH;i++) {
    for (int j=0; j<NB_FRAMES; j++) {
      rx_seqNum_table[i][j] = false;         // Initialize the reception seqNum table
    }
  }
  rx_currentSeqNum = 0;
  
  
  radio.begin();                             // Setup and configure rf radio
  radio.setChannel(ID);                      // Set the RF channel to be equal to the ID (range [0,127])
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(0);                       // Ensure autoACK is disabled - we don't need to know whether the packets are actually received
  radio.setRetries(2,15);                    // Optionally, increase the delay between retries & # of retries
  
  radio.setCRCLength(RF24_CRC_8);            // Use 8-bit CRC for performance

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

  #ifdef DEBUG
    radio.printDetails();                    // Dump the configuration of the rf unit for debugging
  #endif
  #ifdef ROLE_TX
    Serial.print(F("\n\rXI LED Panel TX - USB Serial to RF24 - ID "));
    Serial.println(ID);
  #endif
  #ifdef ROLE_RX
    Serial.print(F("\n\rXI LED Panel RX - RF24 RX to LED Panel - ID "));
    Serial.println(ID);
  #endif

  // Initialize the TX data buffers
  for (int i=0; i<NB_FRAMES; i++) {
    memset(tx_data[i], 0, 32);               // Fill all 32 bytes of tx_data[i] with 0's
    tx_data[i][0] = i*NB_PIXELS_PER_FRAME;   // And put in the the LED offset
  }
  memset(temp_rx_data, 0, 32);               // Do the same for the temp RX buffer
  
  radio.powerUp();                           // Power up the radio

  syncTime = millis();                       // Initialize the PLL sync time
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
        seqNum = (seqNum+1)%RX_QUEUE_DEPTH;    // Increment the ID of the frame currently being sent, while staying in the bounds
        
        #ifdef DEBUG
          Serial.print("FINE: ");
          Serial.print(psize, DEC);    
          Serial.print("/");
          Serial.println(currentPacket, DEC);    
          Serial.send_now();
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
    
    #ifdef DEBUG
      Serial.print("communication_init:");
      Serial.println(communication_init);
    #endif
    
    if (communication_init) {
      update_tx_data_buffers();              // Fill in the TX data buffers
      digitalWrite(ledPin, HIGH);
      long timer = millis();
      send_all_packets();                    // And now send all packets
      Serial.println(millis() - timer);
      digitalWrite(ledPin, LOW);

      }

      if (millis() - tx_last_received_frame_timestamp > TX_COM_REINIT_TIMEOUT) {
        communication_init = false;
        #ifdef DEBUG
          Serial.println("Com Reinit");
          Serial.send_now();
        #endif
      }
    }
  #endif
  
  
  /*************  RX *******************/

  #ifdef ROLE_RX
    while(radio.available()){
     communication_init = true;
     radio.read(&temp_rx_data,32);
     // Check the contents of the received data and put it at the appropriate offset in the frame buffer
     process_rx_data();
     
     #ifdef DEBUG
       Serial.print("RX: ");
       Serial.print(temp_rx_data[0], DEC);
       Serial.print(" / ");
       Serial.print(temp_rx_data[3], DEC);
       Serial.print(" / ");
       Serial.println(temp_rx_data[0 + HEADER_SIZE], DEC);
       Serial.send_now();
     #endif
     rx_last_received_frame_timestamp = millis();
    }

    // Parse the sequence number list, and check if a frame is ready to be sent
    for (int i=0; i<RX_QUEUE_DEPTH; i++) {
      for (int j=0; j<NB_FRAMES; j++) {
        if (!rx_seqNum_table[i][j]) {        // If one of the elements of the table is false, it means that no packet was received for this offset
          break;                             // In that case, no need to continue checking for this particular seqNum, try the next one
        }
        transmit_consolidated_frame(i);
        rxTimer = millis();
        #ifdef DEBUG
          Serial.print("-> TX:");
          Serial.println(rx_packetBuffer[i][0]);
          Serial.send_now();
        #endif
        // Once this frame is sent, consider the job done for this seqnum
        for (int j=0; j<NB_FRAMES; j++) {
          rx_seqNum_table[rx_currentSeqNum][j] = false;         // Reinit the reception seqNum table
        }
        
      }
    }
    
    // Send a consolidated TPM2 frame to the LED microcontroller at least every RX_FRAME_TX_PERIOD_MS milliseconds
    if(millis() - rxTimer > RX_FRAME_TX_PERIOD_MS && communication_init) {
      rxTimer = millis();

      #ifdef DEBUG
        Serial.print("-> Default TX:");
        Serial.send_now();
        Serial.print(rx_currentSeqNum,DEC);
        Serial.print("/");
        Serial.println(rx_packetBuffer[rx_currentSeqNum][0]);
        Serial.send_now();
      #endif
      transmit_consolidated_frame((rx_currentSeqNum + RX_QUEUE_DEPTH-1)%RX_QUEUE_DEPTH);   // Do not send the very last message received, instead send the previous one, which is more likely to be almost complete
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
  if (!communication_init) {
    for (int i=0; i<NB_FRAMES; i++) {
      tx_data[i][1] = s1;                    // Put in the header of the frames to send the MSB of the frame's size
      tx_data[i][2] = s2;                    // Put in the header of the frames to send the LSB of the frame's size
    }
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

  communication_init = true;
  tx_last_received_frame_timestamp = millis();
  
  return psize;
}

/*************  Update the data to send *******************/
void update_tx_data_buffers() {
  for (int i=0; i<NB_FRAMES; i++) {
    memcpy(tx_data[i] + HEADER_SIZE, packetBuffer + i*USEFUL_PAYLOAD_SIZE, USEFUL_PAYLOAD_SIZE);
    tx_data[i][3] = seqNum;
  }
}

void process_rx_data() {
  // temp_rx_data[0] is the LED offset, temp_rx_data[3] is the seqNum
  // Before anything, ensure that the data is not gibberish/invalid
  rx_currentSeqNum = temp_rx_data[3];
  
  if (temp_rx_data[0] >= NB_LEDS || temp_rx_data[0] < 0 || temp_rx_data[3] < 0 || temp_rx_data[3] >= RX_QUEUE_DEPTH) {
    #ifdef DEBUG
      Serial.print("Invalid RX. LED offset:");
      Serial.print(temp_rx_data[0], DEC);
      Serial.print(" / SeqNum");
      Serial.println(temp_rx_data[3], DEC);
      Serial.send_now();
    #endif
    return;
  }
  
  // Copy the received data in the corresponding seqNum buffer, at the correct LED offset
  memcpy(rx_packetBuffer[temp_rx_data[3]] + temp_rx_data[0], temp_rx_data + HEADER_SIZE, USEFUL_PAYLOAD_SIZE);
  frameSize_msb         = temp_rx_data[1];
  frameSize_lsb         = temp_rx_data[2];
  frameSize             = (uint16_t) frameSize_msb << 8;
  frameSize            |= frameSize_lsb;     // Note: this frame size is the total frame size, not the size of this particular packet
  
  // For the current frame seqNum, memorize that a packet was received for this particular LED offset
  rx_seqNum_table[temp_rx_data[3]][int(temp_rx_data[0]/NB_PIXELS_PER_FRAME)] = true;
}

void send_all_packets() {
  for (int i=0; i<NB_FRAMES; i++) {
    // Redundancy trick: when packet loss occurs, it is generally almost at the same time.
    // Rather than sending twice the same package, send the beginning and the end of the frame
    radio.writeFast(&tx_data[i],32);
    //radio.writeFast(&tx_data[NB_FRAMES-1-i],32);      // We might not have the time to do it after all

    #ifdef DEBUG
      Serial.print("RF-TX:");
      Serial.print(tx_data[i][0],DEC);
      Serial.print("/");
      Serial.println(tx_data[i][3],DEC);
      Serial.send_now();
    #endif
    
    if(millis() - syncTime > 3) {          // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = millis();                 // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      radio.txStandBy();                   // This gives the PLL time to sync back up
      delayMicroseconds(200);              // This is absolutely necessary, else the antenna will become entirely unoperational
      #ifdef DEBUG
        Serial.println("Radio txStandby");
        Serial.send_now();
      #endif
    }
  }
}

void transmit_consolidated_frame(uint8_t seqNumber) {
  
  HWSERIAL.write(TPM2NET_HEADER_IDENT);
  HWSERIAL.write(TPM2NET_CMD_DATAFRAME);
  HWSERIAL.write(frameSize_msb);
  HWSERIAL.write(frameSize_lsb);
  HWSERIAL.write(rx_packetnumber);           // Todo: implement multi-packet transmission, by adding the info in the header
  HWSERIAL.write(rx_totalpackets);           // of the frames sent by the Strobot multicontroller

  /*for (int i=0; i<MAX_PACKET_SIZE; i++) {    // Write the contents of the buffer
    HWSERIAL.write(packetBuffer[i]);
  }*/
  HWSERIAL.write(rx_packetBuffer[seqNumber], frameSize);   // Write the contents of the buffer
  
  HWSERIAL.write(TPM2NET_FOOTER_IDENT);

}

