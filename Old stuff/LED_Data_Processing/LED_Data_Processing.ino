
#include <FastSPI_LED2.h>

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
 */


#define NUM_LEDS 20

CRGB leds[NUM_LEDS];

int k=0, jj=0;

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
    LEDS.show();
  }
}



// --------------------------------------------
//      setup
// --------------------------------------------
void setup() {
  delay(2000);
  LEDS.setBrightness(64);


  LEDS.addLeds<WS2801>(leds, NUM_LEDS);

//  FastSPI_LED.setLeds(NUM_LEDS);
//  FastSPI_LED.setChipset(CFastSPI_LED::SPI_WS2801);

  //select spi speed, 7 is very slow, 0 is blazing fast
  //hint: the small (1 led, 5v) spi modules can run maximal at speed2!
  //FastSPI_LED.setDataRate(2);
  //FastSPI_LED.init();
  //FastSPI_LED.start();
  leds = (struct CRGB*)FastSPI_LED.getRGBData(); 

  rainbow();      // display some colors
}

// --------------------------------------------
//      main loop
// --------------------------------------------
void loop() {
  rainbow();    
  
  delay(50);	
}


