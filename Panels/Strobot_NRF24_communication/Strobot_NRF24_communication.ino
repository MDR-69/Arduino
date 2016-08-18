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

/*************  Data compression settings  **********************/
//#define ENABLE_COMPRESSION     0       // If this is defined, every pixel data is mapped down, in order to send fewer frames
//#define COMPRESSION_RATE       2       // Each pixel component is mapped as half a byte

/*************  Hardware Definitions - General  *****************/
/*#ifdef ENABLE_COMPRESSION
  #define NB_LEDS                128
  #define BYTES_PER_PIXEL        3
  #define USEFUL_PAYLOAD_SIZE    24
  #define RF_FRAME_SIZE          32      // Hardware limitation of RF24 modules
  #define HEADER_SIZE            RF_FRAME_SIZE - USEFUL_PAYLOAD_SIZE
  #define NB_FRAMES              int(ceil(NB_LEDS*BYTES_PER_PIXEL/USEFUL_PAYLOAD_SIZE*COMPRESSION_RATE))
  #define NB_PIXELS_PER_FRAME    int(COMPRESSION_RATE*USEFUL_PAYLOAD_SIZE/BYTES_PER_PIXEL)
#else*/
  #define NB_LEDS                128
  #define BYTES_PER_PIXEL        3
  #define USEFUL_PAYLOAD_SIZE    24      
  #define RF_FRAME_SIZE          32      // 32 is the hardware limitation of RF24 modules, but data transmission is more reliable with smaller packets
  #define HEADER_SIZE            RF_FRAME_SIZE - USEFUL_PAYLOAD_SIZE
  #define NB_FRAMES              int(ceil(NB_LEDS*BYTES_PER_PIXEL/(USEFUL_PAYLOAD_SIZE)))
  #define NB_PIXELS_PER_FRAME    int(USEFUL_PAYLOAD_SIZE/BYTES_PER_PIXEL)
//#endif

/*************  Hardware Definitions - TX side only  ************/
#define TX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

/*************  Hardware Definitions - RX side only  ************/
#define NB_FRAMES_TX_PER_SEC   80      // Only used on the RX side: number of frames the receiver shall send to the LED microcontroller per second
#define RX_FRAME_TX_PERIOD_MS  int(1000/NB_FRAMES_TX_PER_SEC)
#define HWSERIAL               Serial1 // UART serial used to forward data to the LED microcontroller
#define HWSERIAL_BAUDRATE      600000  // UART Serial baudrate - equal to 96MHz/(10*16)
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

/*************  RX side data information  *********************/
uint8_t frameSize_msb;
uint8_t frameSize_lsb;
uint16_t frameSize = 0x0000;
const uint8_t rx_packetnumber = 0;      // Todo: implement multi-packet transmission
const uint8_t rx_totalpackets = 1;

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
    HWSERIAL.begin(HWSERIAL_BAUDRATE);                  // Initialize the second Serial link
    HWSERIAL.flush();
    HWSERIAL.setTimeout(20);
  #endif

  memset(packetBuffer, 0, MAX_PACKET_SIZE);  // Initialize the buffer to hold incoming packet data

  radio.begin();                             // Setup and configure rf radio
  radio.setChannel(ID);                      // Set the RF channel to be equal to the ID (range [0,127])
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicAck();                  // This MUST be called prior to attempting single write NOACK calls
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
    Serial.print(F("\n\rXI LED Panel - USB Serial to RF24 - ID "));
    Serial.println(ID);
  #endif
  #ifdef ROLE_RX
    Serial.print(F("\n\rXI LED Panel - RF24 RX to LED Panel - ID "));
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
        #ifdef DEBUG
          Serial.print("FINE: ");
          Serial.print(psize, DEC);    
          Serial.print("/");
          Serial.print(currentPacket, DEC);    
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
    
    if (communication_init) {
      update_tx_data_buffers();              // Fill in the TX data buffers
      //digitalWrite(ledPin, HIGH);
      #ifdef DEBUG
        long timer = millis();
      #endif
      send_all_packets();                    // And now send all packets
      #ifdef DEBUG
        Serial.println(millis() - timer);
      #endif
      //digitalWrite(ledPin, LOW);

      if (millis() - tx_last_received_frame_timestamp > TX_COM_REINIT_TIMEOUT) {
        communication_init = false;
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
  }
}

void process_rx_data() {
  memcpy(packetBuffer + temp_rx_data[0]*BYTES_PER_PIXEL, temp_rx_data + HEADER_SIZE, USEFUL_PAYLOAD_SIZE);
  frameSize_msb = temp_rx_data[1];
  frameSize_lsb = temp_rx_data[2];
  frameSize = (uint16_t) frameSize_msb << 8;
  //frameSize = frameSize << 8;
  frameSize |= frameSize_lsb;             // Note: this frame size is the total frame size, not the size of this particular packet
}

void send_all_packets() {
  #ifdef DEBUG
    Serial.print("RF-TX:");
  #endif
  for (int i=0; i<NB_FRAMES; i++) {
    // Redundancy trick: when packet loss occurs, it is generally almost at the same time.
    // Rather than sending twice the same package, send the beginning and the end of the frame
    radio.writeFast(&tx_data[i],32, 1);    // Do not ask for an ACK when writing this packet
    
    //radio.writeFast(&tx_data[NB_FRAMES-1-i],32, 1);
    
    #ifdef DEBUG
      //Serial.print(i,DEC);
      //Serial.print("/");
      //Serial.print(tx_data[i][0]);
    #endif

    if(millis() - syncTime > 3) {          // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = millis();                 // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      bool ok = radio.txStandBy();         // This gives the PLL time to sync back up
      delayMicroseconds(200);
      #ifdef DEBUG
        Serial.print("txStandby-");
        Serial.println(ok);
        Serial.send_now();
      #endif
    }
  }

  //bool ok = radio.txStandBy();
  
  #ifdef DEBUG
    Serial.println("-->RF-RX over");
  #endif
  
}

void transmit_consolidated_frame() {
  HWSERIAL.write(TPM2NET_HEADER_IDENT);
  HWSERIAL.write(TPM2NET_CMD_DATAFRAME);
  HWSERIAL.write(frameSize_msb);
  HWSERIAL.write(frameSize_lsb);
  //HWSERIAL.flush();
  HWSERIAL.write(rx_packetnumber);           // Todo: implement multi-packet transmission, by adding the info in the header
  HWSERIAL.write(rx_totalpackets);           // of the frames sent by the Strobot multicontroller
  //HWSERIAL.flush();
  for (int i=0; i<frameSize; i++) {          // Write the contents of the buffer
    HWSERIAL.write(packetBuffer[i]);
  }
  //HWSERIAL.write(packetBuffer, frameSize);   // Write the contents of the buffer
  
  HWSERIAL.write(TPM2NET_FOOTER_IDENT);
  //HWSERIAL.flush();

  #ifdef DEBUG
    Serial.print("TXLED: IDENT:");
    Serial.print(TPM2NET_HEADER_IDENT, DEC);
    Serial.print(". CMD:");
    Serial.print(TPM2NET_CMD_DATAFRAME, DEC);
    Serial.print(". Size MSB:");
    Serial.print(frameSize_msb, DEC);
    Serial.print(". Size LSB:");
    Serial.print(frameSize_lsb, DEC);
    Serial.print(". Size:");
    Serial.print((frameSize_msb<<8) + frameSize_lsb, DEC);
    Serial.print(". RealSize:");
    Serial.print(frameSize, DEC);
    Serial.print(". rx_packetnumber:");
    Serial.print(rx_packetnumber, DEC);
    Serial.print(". rx_totalpackets:");
    Serial.print(rx_totalpackets, DEC);
    Serial.print(". packetBuffer[0]:");
    Serial.print(packetBuffer[0], DEC);
    Serial.print(". packetBuffer[1]:");
    Serial.println(packetBuffer[1], DEC);
  #endif
  
}

