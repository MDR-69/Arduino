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

//#define DEBUG                  1

/*************  Hardware Definitions *******************************/
#define ID                     3        // Define the ID of the panel to communicate with, range is [0,4]
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
#define USEFUL_PAYLOAD_SIZE    9       // Each packet contains the data for a few leds LEDs, and a header      
#define RF_FRAME_SIZE          10      // 32 is the hardware limitation of RF24 modules, but data transmission is more reliable with smaller packets
#define DATA_BUFFER_SIZE       NB_LEDS*BYTES_PER_PIXEL
#define HEADER_SIZE            RF_FRAME_SIZE - USEFUL_PAYLOAD_SIZE
#define NB_FRAMES              int(ceil(NB_LEDS*BYTES_PER_PIXEL/(USEFUL_PAYLOAD_SIZE + 0.0)))
#define NB_PIXELS_PER_FRAME    int(USEFUL_PAYLOAD_SIZE/BYTES_PER_PIXEL)
//#define FRAMESIZE_IN_HEADER  1       // Best not to include the frame size in the header, as it allows to shave off 2 bytes from the packet
#define SHORT_RF_ADDRESS       1       // Use 24 bit addresses instead of 40 bit ones
#define ENABLE_CRC             1       // Use 8 bit CRC for performance. Not much is gained from setting it off, and it prevents from receiving gibberish

/*************  Hardware Definitions - TX side only  ************/
#define TX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

/*************  Hardware Definitions - RX side only  ************/
#define NB_FRAMES_TX_PER_SEC   80      // Only used on the RX side: number of frames the receiver shall send to the LED microcontroller per second
#define RX_FRAME_TX_PERIOD_MS  int(1000/NB_FRAMES_TX_PER_SEC)
#define HWSERIAL               Serial1 // UART serial used to forward data to the LED microcontroller
#define HWSERIAL_BAUDRATE      6000000 // UART Serial baudrate - equal to 96MHz/16, absolute maximum available with Teensy 3.2
#define RX_COM_REINIT_TIMEOUT  20000   // If no frame is received after 20 seconds, consider the communication link to be down

//#define OPTIMIZE_LED_FRAME_TX  1     // When a frame must be transmitted to the LED microcontroller, send it using non-blocking Serial write calls
                                       // -- This is actually not necessary, another way around the problem exists:
                                       // Teensy's Serial1 RX/TX buffer sizes can be modified
                                       // In /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3, change in the file serial1.c :
                                       // #define TX_BUFFER_SIZE     64 // number of outgoing bytes to buffer     ---> Change to 512
                                       // #define RX_BUFFER_SIZE     64 // number of incoming bytes to buffer     ---> Change to 512
                                       // With the buffers this big, the HWSERIAL.write call becomes non-blocking, and returns after 140 microsecs (instead of 580)
#define SAME_COUNTER_PACKET_THR 400   // If the same packet is received N consecutive times, reset the system
#define CPU_RESTART_ADDR        (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL         0x5FA0004
#define CPU_RESTART             (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
                                       
/*************  TPM2 Definitions *******************************/
#define TPM2NET_HEADER_SIZE 4
#define TPM2NET_HEADER_IDENT 0x9c
#define TPM2NET_CMD_DATAFRAME 0xda
#define TPM2NET_CMD_COMMAND 0xc0
#define TPM2NET_CMD_ANSWER 0xaa
#define TPM2NET_FOOTER_IDENT 0x36
#define SERIAL_FOOTER_SIZE 1

#define SPECIAL_FULLFRAME_OFFSET 0xFF  // Special case: if the received frame is entirely of the same color, only one packet is enough to tell this to the RX

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
uint8_t frameSize_msb;                  // Not used with the current protocol
uint8_t frameSize_lsb;                  // There isn't enough space in the header to put in the frame size
uint16_t frameSize = 0x0000;            // These three variables are only used if FRAMESIZE_IN_HEADER is defined
const uint8_t rx_packetnumber = 0;      // Todo: implement multi-packet transmission
const uint8_t rx_totalpackets = 1;      // Note: given the current performance, it doesn't seem too possible to deal with 170+ LED arrays
const uint8_t predefined_frameSize_msb = (uint8_t) ((NB_LEDS * BYTES_PER_PIXEL) >> 8);
const uint8_t predefined_frameSize_lsb = (uint8_t) ((NB_LEDS * BYTES_PER_PIXEL) & 0xFF);
bool transmit_to_led_teensy = false;    // When the flag is set to true, send a consolidated frame to the next Teensy
uint16_t serialBuffer_write_offset;     // What byte should we be writing next ?
bool special_uniform_frame = false;     // Is the frame entirely of the same color ?
uint32_t same_offset_cpt = 0;           // This counter is used to know how many consecutive packets referring to the same offset have been received (this does not concern the special header values)
uint8_t  same_offset_val = 0;           // Seen during testing: in some rare cases, the antenna switches to a weird mode and transmits again and again the same frame
                                        // Try to detect this case, and correct the problem by powering off the antenna, and resetting  the microcontroller

/*************  Hardware RF Configuration  *********************/
RF24 radio(7,8);                        // Set up nRF24L01 radio on SPI bus plus pins 7 & 8
const int ledPin = 13;                  // Teensy 3.0 has the LED on pin 13

/***************************************************************/

// Radio pipe addresses for the nodes to communicate.
// The five first addresses are the panels', the following are the local emitters' addresses
// These uint64_t addresses are used in the 40-bit address mode. To optimize the system, shave off two bytes and use 24 bit long addresses
const uint64_t pipes[10] = { 0xABCDABCD71LL, 0x544d52687CLL,          // Note: a restriction requires the 1-5 reading pipes to share the 
                             0x544d526832LL, 0x544d52683CLL,          // Same top 32 bits, only to have the LSB change
                             0x544d526846LL, 0x544d526850LL,          // As we do not need to listen on more than one pipe (point to point
                             0x544d52685ALL, 0x544d526820LL,          // communication, free to set very specific addresses)
                             0x544d52686ELL, 0x544d52684BLL};
const uint32_t short_pipes[10] = { 0xABCD71L, 0xABCD72L,
                                   0x143EB6L, 0x143EB7L,
                                   0xCA24F9L, 0xCA24FAL,
                                   0x34830BL, 0x34830CL, 
                                   0x744D6EL, 0x744D6FL};
const uint8_t channels[5] = {6,              // It is important to use channels which are far from each other
                             28,             // As the full bandwidth is used for each TX/RX pair, non-overlapping
                             60,             // frequencies are critical for proper operation
                             89,             // Note: do not use 96, as this is for the LED tubes
                             119};

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
    HWSERIAL.begin(HWSERIAL_BAUDRATE);       // Initialize the second Serial link
    HWSERIAL.flush();
    HWSERIAL.setTimeout(20);
  #endif

  memset(packetBuffer, 0, MAX_PACKET_SIZE);  // Initialize the buffer to hold incoming packet data

  radio.begin();                             // Setup and configure rf radio
  radio.setChannel(channels[ID]);            // Set the RF channels of the different modules wide enough apart not to have interference (range [0,125])
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  //radio.enableDynamicAck();                // This MUST be called prior to attempting single write NOACK calls // No need to to that as ACK is disabled
  radio.setAutoAck(0);                       // Ensure autoACK is disabled - we don't need to know whether the packets are actually received
  //radio.setRetries(2,15);                  // Optionally, increase the delay between retries & # of retries // No need to to that as ACK is disabled

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
    tx_data[i][0] = i*NB_PIXELS_PER_FRAME;   // And put in the LED offset
  }
  memset(temp_rx_data, 0, RF_FRAME_SIZE);    // Do the same for the temp RX buffer
  serialBuffer_write_offset = -1;
  
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
        //send_all_packets();                  // And now send all packets - not a good idea to do it here. If more frames comi in from Strobot than
                                               // what can be transmitted through radio.write, the USB Serial buffers will get filled, and the Teensy will freeze
        
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
      //update_tx_data_buffers();              // Fill in the TX data buffers - if a seqNum is included in the frame, the update must be done here. It is not the case with the minimal header used here.

      //send_all_packets();                    // And now send all packets - don't, too much work at once: this takes a lot of time, and new frames are still
      send_specific_packet();                  // coming in through the USB Serial link. With a higher framerate, the USB rx buffer would eventually get filled,
                                               // and the Teensy would freeze. Instead, send one packet packet at a time, and check if a new frame was received

      // Actually not that good of an idea regarding TX : always try to send data, you never know !
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

      // Check if we're not getting into the weird RX mode, where the antenna only forwards the same packet again and again
      if (same_offset_val == temp_rx_data[0] && temp_rx_data[0] < NB_LEDS) {
        same_offset_cpt += 1;
        if (same_offset_cpt >= SAME_COUNTER_PACKET_THR) {
          reset_system();
        }
      }
      else {
        same_offset_cpt = 0;
      }
      same_offset_val = temp_rx_data[0];
      
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
    // However, sending a full frame at once takes too much time - even with the baudrate set at 6000000, the
    // absolute maximum, it takes about 583 microseconds. This isn't much, but a nRF packet may be received
    // every 100 microseconds - and the FIFO buffer of the nRF24l01 module is only 3 frame long.
    // Set a "get ready to send the frame" flag and use non-blocking serial calls
    if(millis() - rxTimer > RX_FRAME_TX_PERIOD_MS && communication_init) {
      rxTimer = millis();                      // Update the reference TX time
      #ifdef OPTIMIZE_LED_FRAME_TX
        transmit_to_led_teensy = true;         // Set the TX flag to true
        serialBuffer_write_offset = -1;
      #else
        transmit_consolidated_frame();         // Or in non-optimized mode, send everything at once
        #ifdef DEBUG
          Serial.print("-> TX:");
          Serial.println(packetBuffer[0]);
        #endif
      #endif
    }

    #ifdef OPTIMIZE_LED_FRAME_TX
    if (transmit_to_led_teensy) {
      if (serialBuffer_write_offset == -1) {
        transmit_consolidated_frame_header();
      }
      else {
        transmit_consolidated_frame_body();
      }
    }
    #endif

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

  // Check if the currently processed frame is uniform (all pixels are the same color)
  bool sameColor   = true;
  uint8_t redVal   = packetBuffer[0];
  uint8_t greenVal = packetBuffer[1];
  uint8_t blueVal  = packetBuffer[2];
  for (int i=3; i<DATA_BUFFER_SIZE; i+=BYTES_PER_PIXEL) {
    if (!(packetBuffer[i] == redVal && packetBuffer[i+1] == greenVal && packetBuffer[i+2] == blueVal)) {
      sameColor = false;
      break;
    }
  }
  // If the current frame is uniform, a special offset will be used when sending the packets
  special_uniform_frame = sameColor;
  if (sameColor) {
    special_uniform_frame = true;
    for (int i=0; i<NB_FRAMES; i++) {
      tx_data[i][0] = SPECIAL_FULLFRAME_OFFSET;   // Put in the special "full frame" LED offset
    }
  }
  else {
    special_uniform_frame = false;
    for (int i=0; i<NB_FRAMES; i++) {
      tx_data[i][0] = i*NB_PIXELS_PER_FRAME;      // Put in the regular LED offset
    }    
  }
  
}

void process_rx_data() {
  uint8_t offset_val = temp_rx_data[0];
  if (offset_val == SPECIAL_FULLFRAME_OFFSET) {
    for (int i=0; i<DATA_BUFFER_SIZE; i+=3) {
      packetBuffer[i]   = temp_rx_data[HEADER_SIZE];
      packetBuffer[i+1] = temp_rx_data[HEADER_SIZE+1];
      packetBuffer[i+2] = temp_rx_data[HEADER_SIZE+2];
    }
  }
  else {
    memcpy(packetBuffer + offset_val*BYTES_PER_PIXEL, temp_rx_data + HEADER_SIZE, USEFUL_PAYLOAD_SIZE);
  }
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

    if(micros() - syncTime > 3700) {          // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = micros();                 // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      radio.txStandBy();         // This gives the PLL time to sync back up
      //delayMicroseconds(130);
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
  
  if(micros() - syncTime > 3700) {       // This is only required because NO ACK ( enableAutoAck(0) ) payloads are used
      syncTime = millis();               // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
      radio.txStandBy();                 // This gives the PLL time to sync back up
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

void transmit_consolidated_frame_header() {
  HWSERIAL.write(TPM2NET_HEADER_IDENT);
  HWSERIAL.write(TPM2NET_CMD_DATAFRAME);
  HWSERIAL.write(predefined_frameSize_msb);  // Frame size MSB (1)   - not enough space left in the header to include the frame size
  HWSERIAL.write(predefined_frameSize_lsb);  // Frame size LSB (128) - anyways, the size is always the same, as it is hardware-dependant
  serialBuffer_write_offset = 0;
}

void transmit_consolidated_frame_body() {
 uint8_t availableBytes = HWSERIAL.availableForWrite();
}

void transmit_consolidated_frame_footer() {
  HWSERIAL.write(TPM2NET_FOOTER_IDENT);
  transmit_to_led_teensy = false;
}

void reset_system() {
  radio.stopListening();
  radio.powerDown();                        // Power down the radio
  delay(100);                               // Wait 0.1 seconds
  CPU_RESTART                               // And request a software restart from the Teensy
  same_offset_cpt = 0;                      // Unreachable code, here just in case
  delay(50);
}

