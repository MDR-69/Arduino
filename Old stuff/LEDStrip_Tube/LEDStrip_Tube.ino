/*
** Copyright Martin Di Rollo / PlayMe - 2014
**
** LED Strip controller
** This program aims to control WS2812B strips of LED, by getting orders from
** its neighboring Arduino (controlled wirelessly by a master Processing app)
** The received commands allow to switch between the various animations
**
** Important note : all rhythmic animations come as a pair, calling the second
** one resets the parameters and allows re-execution
*/

#include "FastSPI_LED2.h"
#include "printf.h"

#define NUM_LEDS 240

#define MAX_INTENSITY_VALUE 255

#define DATA_PIN 11

//Parameters for the different animations
#define SLOW_SINE_VALUE 1
#define FAST_SINE_VALUE 10
#define SMOOTHSINE_RAINBOW_TIMEFACTOR 40

#define VERYSLOW_STROBO_DELAY 75
#define SLOW_STROBO_DELAY 50
#define MEDIUM_STROBO_DELAY 25
#define FAST_STROBO_DELAY 10

#define FAST_FLASH_DELAY 0
#define SLOW_FLASH_DELAY 2

#define SMOOTH_SINE_DELAY 1
#define SMOOTH_RANDOM_DELAY 1

#define NOISE_DELAY 5

#define FAST_GROWINGSTROBO_SPEEDFACTOR 3
#define MEDIUM_GROWINGSTROBO_SPEEDFACTOR 2
#define SLOW_GROWINGSTROBO_SPEEDFACTOR 1
#define VERYSLOW_GROWINGSTROBO_SPEEDFACTOR 1

#define SHORT_WAVELENGTH 20
#define LONG_WAVELENGTH 60
#define SINGLE_WAVE_DELAY 0

#define FAST_BUILDUP_DELAY 0
#define MEDIUM_BUILDUP_DELAY 4
#define SLOW_BUILDUP_DELAY 15
#define VERYSLOW_BUILDUP_DELAY 50
#define BUILDUP_RAINBOW_TIMEFACTOR 40


CRGB leds[NUM_LEDS];           // the current values of the LED strip
int animationNumber = 1;      // a variable to store the number of the current animation
unsigned int progress = 0;              // a variable to make the animations evolve
int value = 0;                 // a variable to read incoming serial data into

void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  
  // Start a serial communication with the neighboring Arduino
  Serial.begin(57600);  
  printf_begin();
  //printf("\n\rPlayMe Custom Device - LED Tube, awaiting serial input...\n\r");
  
  //Set a 5ms timeout for Serial read
  Serial.setTimeout(5);
  
  // Clear any garbage in the buffer
  Serial.parseInt();
}

void loop() {
  checkIncomingSerial();
  
  switch(animationNumber) {    
    case 0:  blackOut(); break;
    case 1:  whiteOut(); break;
    case 2:  redOut(); break;
    case 3:  blueOut(); break;
    case 4:  rainbowOut();break;
    
    case 5:  fastWhiteFlash(); break;                     //Rhythmic
    case 6:  fastWhiteFlash(); break;
    case 7:  fastRedFlash(); break;                       //Rhythmic
    case 8:  fastRedFlash(); break;
    case 9:  fastBlueFlash(); break;                      //Rhythmic
    case 10: fastBlueFlash(); break;    
    case 11: fastRainbowFlash();break;                    //Rhythmic
    case 12: fastRainbowFlash();break;
    case 13: slowWhiteFlash(); break;                     //Rhythmic
    case 14: slowWhiteFlash(); break;
    case 15: slowRedFlash(); break;                       //Rhythmic
    case 16: slowRedFlash(); break;
    case 17: slowBlueFlash(); break;                      //Rhythmic
    case 18: slowBlueFlash(); break;
    case 19: slowRainbowFlash();break;                    //Rhythmic
    case 20: slowRainbowFlash();break;
    
    case 21: slowWhiteSmoothSine(); break;
    case 22: slowRedSmoothSine(); break;
    case 23: slowBlueSmoothSine(); break;
    case 24: slowRainbowSmoothSine(); break;    
    case 25: fastWhiteSmoothSine(); break;
    case 26: fastRedSmoothSine(); break;
    case 27: fastBlueSmoothSine(); break;
    case 28: fastRainbowSmoothSine(); break;    
    
    case 29: fastWhiteStroboscope(); break;
    case 30: fastRedStroboscope(); break;
    case 31: fastBlueStroboscope(); break;
    case 32: fastRandomStroboscope(); break;
    case 33: fastGrowingWhiteStroboscope(); break;        //Rhythmic
    case 34: fastGrowingWhiteStroboscope(); break;    
    case 35: fastGrowingRedStroboscope(); break;          //Rhythmic
    case 36: fastGrowingRedStroboscope(); break;

    case 37: mediumWhiteStroboscope(); break;
    case 38: mediumRedStroboscope(); break;
    case 39: mediumBlueStroboscope(); break;
    case 40: mediumRandomStroboscope(); break;
    case 41: mediumGrowingWhiteStroboscope(); break;      //Rhythmic
    case 42: mediumGrowingWhiteStroboscope(); break;    
    case 43: mediumGrowingRedStroboscope(); break;        //Rhythmic
    case 44: mediumGrowingRedStroboscope(); break;

    case 45: slowWhiteStroboscope(); break;
    case 46: slowRedStroboscope(); break;
    case 47: slowBlueStroboscope(); break;
    case 48: slowRandomStroboscope(); break;
    case 49: slowGrowingWhiteStroboscope(); break;        //Rhythmic
    case 50: slowGrowingWhiteStroboscope(); break;    
    case 51: slowGrowingRedStroboscope(); break;          //Rhythmic
    case 52: slowGrowingRedStroboscope(); break;
    
    case 53: veryslowWhiteStroboscope(); break;
    case 54: veryslowRedStroboscope(); break;
    case 55: veryslowBlueStroboscope(); break;
    case 56: veryslowRandomStroboscope(); break;
    case 57: veryslowGrowingWhiteStroboscope(); break;    //Rhythmic
    case 58: veryslowGrowingWhiteStroboscope(); break;    
    case 59: veryslowGrowingRedStroboscope(); break;      //Rhythmic
    case 60: veryslowGrowingRedStroboscope(); break;    
    
    case 61: singleWhiteShortUpwardWave(); break;         //Rhythmic
    case 62: singleWhiteShortUpwardWave(); break;
    case 63: singleRedShortUpwardWave(); break;           //Rhythmic
    case 64: singleRedShortUpwardWave(); break;
    case 65: singleBlueShortUpwardWave(); break;          //Rhythmic
    case 66: singleBlueShortUpwardWave(); break;
    case 67: singleWhiteLongUpwardWave(); break;          //Rhythmic
    case 68: singleWhiteLongUpwardWave(); break;
    case 69: singleRedLongUpwardWave(); break;            //Rhythmic
    case 70: singleRedLongUpwardWave(); break;
    case 71: singleBlueLongUpwardWave(); break;           //Rhythmic
    case 72: singleBlueLongUpwardWave(); break;

    case 73: singleWhiteShortDownwardWave(); break;       //Rhythmic
    case 74: singleWhiteShortDownwardWave(); break;
    case 75: singleRedShortDownwardWave(); break;         //Rhythmic
    case 76: singleRedShortDownwardWave(); break;
    case 77: singleBlueShortDownwardWave(); break;        //Rhythmic
    case 78: singleBlueShortDownwardWave(); break;
    case 79: singleWhiteLongDownwardWave(); break;        //Rhythmic
    case 80: singleWhiteLongDownwardWave(); break;
    case 81: singleRedLongDownwardWave(); break;          //Rhythmic
    case 82: singleRedLongDownwardWave(); break;
    case 83: singleBlueLongDownwardWave(); break;         //Rhythmic
    case 84: singleBlueLongDownwardWave(); break;
    
    case 85: fastWhiteBuildup(); break;                   //Rhythmic
    case 86: fastWhiteBuildup(); break;
    case 87: fastRedBuildup(); break;                     //Rhythmic
    case 88: fastRedBuildup(); break;
    case 89: fastBlueBuildup(); break;                    //Rhythmic
    case 90: fastBlueBuildup(); break;
    case 91: fastRainbowBuildup(); break;                 //Rhythmic
    case 92: fastRainbowBuildup(); break;
    
    case 93: mediumWhiteBuildup(); break;                 //Rhythmic
    case 94: mediumWhiteBuildup(); break;
    case 95: mediumRedBuildup(); break;                   //Rhythmic
    case 96: mediumRedBuildup(); break;
    case 97: mediumBlueBuildup(); break;                  //Rhythmic
    case 98: mediumBlueBuildup(); break;
    case 99: mediumRainbowBuildup(); break;               //Rhythmic
    case 100: mediumRainbowBuildup(); break;

    case 101: slowWhiteBuildup(); break;                  //Rhythmic
    case 102: slowWhiteBuildup(); break;
    case 103: slowRedBuildup(); break;                    //Rhythmic
    case 104: slowRedBuildup(); break;
    case 105: slowBlueBuildup(); break;                   //Rhythmic
    case 106: slowBlueBuildup(); break;
    case 107: slowRainbowBuildup(); break;                //Rhythmic
    case 108: slowRainbowBuildup(); break;

    case 109: veryslowWhiteBuildup(); break;              //Rhythmic
    case 110: veryslowWhiteBuildup(); break;
    case 111: veryslowRedBuildup(); break;                //Rhythmic
    case 112: veryslowRedBuildup(); break;
    case 113: veryslowBlueBuildup(); break;               //Rhythmic
    case 114: veryslowBlueBuildup(); break;
    case 115: veryslowRainbowBuildup(); break;            //Rhythmic
    case 116: veryslowRainbowBuildup(); break;
    
    case 117: whiteNoise();break;
    case 118: redNoise();break;
    case 119: blueNoise();break;
    case 120: rainbowNoise();break;
    
    case 121: slowWhiteSmoothNoise(); break;
    case 122: slowRedSmoothNoise(); break;
    case 123: slowBlueSmoothNoise(); break;
    case 124: slowRainbowSmoothNoise(); break;    
    case 125: fastWhiteSmoothNoise(); break;
    case 126: fastRedSmoothNoise(); break;
    case 127: fastBlueSmoothNoise(); break;
    case 128: fastRainbowSmoothNoise(); break;      
    
    default: whiteOut(); break;        //If the animation number is invalid, play it safe, whiteout (for debug purposes - later, change it to blackout)
  }
}


void checkIncomingSerial() {
  // see if there's incoming serial data:
  int c;
//  if (Serial.available() >= 0) {
//    //printf("Serial.available() : %d\n", Serial.available());
//    c = Serial.read();
//    
//    if ((c>='0') && (c<='9')) {
//      value = 10*value + c - '0';
//    } else {
//      if (c=='W') {
//          printf("Received command : animation %d\n", value);
//        
//        // If the incoming command is different from the current animation, reset all progress variables
//        if (value != animationNumber) {
//          animationNumber = value;
//          setupAnimation();
//        }
//        value = 0;
//      }
//    }
//  }
  if (Serial.available() > 0) {
    c = Serial.parseInt();

    printf("Received command : animation %d\n", c);
    
    // If the incoming command is different from the current animation, reset all progress variables
    if (c != animationNumber) {
      animationNumber = c;
      setupAnimation();
    
      c = 0;
    }
    
  }

  
}

//Reinitialize all parameters
void setupAnimation() {
  progress = 0;
}

////////////////////
//Homogeneous colors

void blackOut() {
  FastLED.showColor(CRGB(0,0,0));
}

void whiteOut() {
  FastLED.showColor(CRGB(MAX_INTENSITY_VALUE,MAX_INTENSITY_VALUE,MAX_INTENSITY_VALUE));
}

void redOut() {
  FastLED.showColor(CRGB(MAX_INTENSITY_VALUE,0,0));
}

void blueOut() {
  FastLED.showColor(CRGB(0,0,MAX_INTENSITY_VALUE));
}

void rainbowOut() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);  
  FastLED.showColor(CRGB(colorWheel.r, colorWheel.g, colorWheel.b));  
}


//////////////////////
//Homogeneous flash

void fastWhiteFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);
  FastLED.showColor(CRGB(value, value, value));
  fastFlashDelay();
  if (progress < 255) {
    progress += 4;
  }
}

void fastRedFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);
  FastLED.showColor(CRGB(value, 0, 0));
  fastFlashDelay();
  if (progress < 255) {
    progress += 4;
  }
}

void fastBlueFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);
  FastLED.showColor(CRGB(0, 0, value));
  slowFlashDelay();
  if (progress < 255) {
    progress += 4;
  }
}

void fastRainbowFlash() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);
  FastLED.showColor(CRGB(max(colorWheel.r-((int)progress), 0), max(colorWheel.g-((int)progress), 0), max(colorWheel.b-((int)progress), 0)));
  fastFlashDelay();
  if (progress < 255) {
    progress += 4;
  }
}

void slowWhiteFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);  
  FastLED.showColor(CRGB(value, value, value));
  slowFlashDelay();
  if (progress < 255) {
    progress += 1;
  }
}

void slowRedFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);  
  FastLED.showColor(CRGB(value, 0, 0));
  slowFlashDelay();
  if (progress < 255) {
    progress += 1;
  }
}

void slowBlueFlash() {
  int value = max(MAX_INTENSITY_VALUE-(int)progress,0);  
  FastLED.showColor(CRGB(0, 0, value));
  slowFlashDelay();
  if (progress < 255) {
    progress += 1;
  }
}

void slowRainbowFlash() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);  
  FastLED.showColor(CRGB(max(colorWheel.r-((int)progress), 0), max(colorWheel.g-((int)progress), 0), max(colorWheel.b-((int)progress), 0)));
  slowFlashDelay();
  if (progress < 255) {
    progress += 1;
  }
}

//////////////////////////
// Sine animations (wave)

void fastWhiteSmoothSine() {
  smoothSine(FAST_SINE_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void fastRedSmoothSine() {
  smoothSine(FAST_SINE_VALUE, MAX_INTENSITY_VALUE, 0, 0);
}

void fastBlueSmoothSine() {
  smoothSine(FAST_SINE_VALUE, 0, 0, MAX_INTENSITY_VALUE);
}

void fastRainbowSmoothSine() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);
  smoothSine(FAST_SINE_VALUE, colorWheel.r, colorWheel.g, colorWheel.b);
}

void slowWhiteSmoothSine() {
  smoothSine(SLOW_SINE_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);  //To be tested
}

void slowRedSmoothSine() {
  smoothSine(SLOW_SINE_VALUE, MAX_INTENSITY_VALUE, 0, 0);
}

void slowBlueSmoothSine() {
  smoothSine(SLOW_SINE_VALUE, 0, 0, MAX_INTENSITY_VALUE);
}

void slowRainbowSmoothSine() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);
  smoothSine(SLOW_SINE_VALUE, colorWheel.r, colorWheel.g, colorWheel.b);
}

//////////////////////////
// Stroboscope animations

void fastWhiteStroboscope() {
  stroboscope(FAST_STROBO_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void fastRedStroboscope() {
  stroboscope(FAST_STROBO_DELAY, MAX_INTENSITY_VALUE, 0, 0);
}

void fastBlueStroboscope() {
  stroboscope(FAST_STROBO_DELAY, 0, 0, MAX_INTENSITY_VALUE);
}

void fastRandomStroboscope() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));
  stroboscope(FAST_STROBO_DELAY, randomColor.r, randomColor.g, randomColor.b);
}

void fastGrowingWhiteStroboscope() {
  int intensity = min(progress*FAST_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(FAST_STROBO_DELAY, intensity, intensity, intensity);  
  progress += 1;
}

void fastGrowingRedStroboscope() {
  int intensity = min(progress*FAST_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(FAST_STROBO_DELAY, intensity, 0, 0);  
  progress += 1;
}

void mediumWhiteStroboscope() {
  stroboscope(MEDIUM_STROBO_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void mediumRedStroboscope() {
  stroboscope(MEDIUM_STROBO_DELAY, MAX_INTENSITY_VALUE, 0, 0);
}

void mediumBlueStroboscope() {
  stroboscope(MEDIUM_STROBO_DELAY, 0, 0, MAX_INTENSITY_VALUE);
}

void mediumRandomStroboscope() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));      //A tester
  stroboscope(MEDIUM_STROBO_DELAY, randomColor.r, randomColor.g, randomColor.b);
}

void mediumGrowingWhiteStroboscope() {
  int intensity = min(progress*MEDIUM_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(MEDIUM_STROBO_DELAY, intensity, intensity, intensity);  
  progress += 1;
}

void mediumGrowingRedStroboscope() {
  int intensity = min(progress*MEDIUM_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(MEDIUM_STROBO_DELAY, intensity, 0, 0);  
  progress += 1;
}

void slowWhiteStroboscope() {
  stroboscope(SLOW_STROBO_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void slowRedStroboscope() {
  stroboscope(SLOW_STROBO_DELAY, MAX_INTENSITY_VALUE, 0, 0);
}

void slowBlueStroboscope() {
  stroboscope(SLOW_STROBO_DELAY, 0, 0, MAX_INTENSITY_VALUE);
}

void slowRandomStroboscope() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));      //A tester
  stroboscope(SLOW_STROBO_DELAY, randomColor.r, randomColor.g, randomColor.b);
}

void slowGrowingWhiteStroboscope() {
  int intensity = min(progress*SLOW_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(SLOW_STROBO_DELAY, intensity, intensity, intensity);  
  progress += 1;
}

void slowGrowingRedStroboscope() {
  int intensity = min(progress*SLOW_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(SLOW_STROBO_DELAY, intensity, 0, 0);  
  progress += 1;
}

void veryslowWhiteStroboscope() {
  stroboscope(VERYSLOW_STROBO_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void veryslowRedStroboscope() {
  stroboscope(VERYSLOW_STROBO_DELAY, MAX_INTENSITY_VALUE, 0, 0);
}

void veryslowBlueStroboscope() {
  stroboscope(VERYSLOW_STROBO_DELAY, 0, 0, MAX_INTENSITY_VALUE);
}

void veryslowRandomStroboscope() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));      //A tester
  stroboscope(VERYSLOW_STROBO_DELAY, randomColor.r, randomColor.g, randomColor.b);
}

void veryslowGrowingWhiteStroboscope() {
  int intensity = min(progress*VERYSLOW_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(VERYSLOW_STROBO_DELAY, intensity, intensity, intensity);  
  progress += 1;
}

void veryslowGrowingRedStroboscope() {
  int intensity = min(progress*VERYSLOW_GROWINGSTROBO_SPEEDFACTOR, MAX_INTENSITY_VALUE);
  stroboscope(VERYSLOW_STROBO_DELAY, intensity, 0, 0);  
  progress += 1;
}


//////////////////////////
// SingleWave animations

void singleWhiteShortUpwardWave() {
  upwardWave(SHORT_WAVELENGTH, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void singleRedShortUpwardWave() {
  upwardWave(SHORT_WAVELENGTH, MAX_INTENSITY_VALUE, 0, 0);
}

void singleBlueShortUpwardWave() {
  upwardWave(SHORT_WAVELENGTH, 0, 0, MAX_INTENSITY_VALUE);
}

void singleRandomShortUpwardWave() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));  
  upwardWave(SHORT_WAVELENGTH, randomColor.r, randomColor.g, randomColor.b);
}

void singleWhiteShortDownwardWave() {
  downwardWave(SHORT_WAVELENGTH, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void singleRedShortDownwardWave() {
  downwardWave(SHORT_WAVELENGTH, MAX_INTENSITY_VALUE, 0, 0);
}

void singleBlueShortDownwardWave() {
  downwardWave(SHORT_WAVELENGTH, 0, 0, MAX_INTENSITY_VALUE);
}

void singleRandomShortDownwardWave() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));  
  downwardWave(SHORT_WAVELENGTH, randomColor.r, randomColor.g, randomColor.b);
}


void singleWhiteLongUpwardWave() {
  upwardWave(LONG_WAVELENGTH, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void singleRedLongUpwardWave() {
  upwardWave(LONG_WAVELENGTH, MAX_INTENSITY_VALUE, 0, 0);
}

void singleBlueLongUpwardWave() {
  upwardWave(LONG_WAVELENGTH, 0, 0, MAX_INTENSITY_VALUE);
}

void singleRandomLongUpwardWave() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));  
  upwardWave(LONG_WAVELENGTH, randomColor.r, randomColor.g, randomColor.b);
}

void singleWhiteLongDownwardWave() {
  downwardWave(LONG_WAVELENGTH, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void singleRedLongDownwardWave() {
  downwardWave(LONG_WAVELENGTH, MAX_INTENSITY_VALUE, 0, 0);
}

void singleBlueLongDownwardWave() {
  downwardWave(LONG_WAVELENGTH, 0, 0, MAX_INTENSITY_VALUE);
}

void singleRandomLongDownwardWave() {
  CRGB randomColor;
  randomColor.CRGB::setHue((int)(MAX_INTENSITY_VALUE*random()));  
  downwardWave(LONG_WAVELENGTH, randomColor.r, randomColor.g, randomColor.b);
}


//////////////////////////
// Buildup animations

void fastWhiteBuildup() {
  buildup(FAST_BUILDUP_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
  progress += 1;
}

void mediumWhiteBuildup() {
  buildup(MEDIUM_BUILDUP_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
  progress += 1;
}

void slowWhiteBuildup() {
  buildup(SLOW_BUILDUP_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
  progress += 1;
}

void veryslowWhiteBuildup() {
  buildup(VERYSLOW_BUILDUP_DELAY, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
  progress += 1;
}

void fastRedBuildup() {
  buildup(FAST_BUILDUP_DELAY, MAX_INTENSITY_VALUE, 0, 0);
  progress += 1;
}

void mediumRedBuildup() {
  buildup(MEDIUM_BUILDUP_DELAY, MAX_INTENSITY_VALUE, 0, 0);
  progress += 1;
}

void slowRedBuildup() {
  buildup(SLOW_BUILDUP_DELAY, MAX_INTENSITY_VALUE, 0, 0);
  progress += 1;
}

void veryslowRedBuildup() {
  buildup(VERYSLOW_BUILDUP_DELAY, MAX_INTENSITY_VALUE, 0, 0);
  progress += 1;
}

void fastBlueBuildup() {
  buildup(FAST_BUILDUP_DELAY, 0, 0, MAX_INTENSITY_VALUE);
  progress += 1;
}

void mediumBlueBuildup() {
  buildup(MEDIUM_BUILDUP_DELAY, 0, 0, MAX_INTENSITY_VALUE);
  progress += 1;
}

void slowBlueBuildup() {
  buildup(SLOW_BUILDUP_DELAY, 0, 0, MAX_INTENSITY_VALUE);
  progress += 1;
}

void veryslowBlueBuildup() {
  buildup(VERYSLOW_BUILDUP_DELAY, 0, 0, MAX_INTENSITY_VALUE);
  progress += 1;
}

void fastRainbowBuildup() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / BUILDUP_RAINBOW_TIMEFACTOR) % 255);
  buildup(FAST_BUILDUP_DELAY, colorWheel.r, colorWheel.g, colorWheel.b);
  progress += 1;
}

void mediumRainbowBuildup() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / BUILDUP_RAINBOW_TIMEFACTOR) % 255);
  buildup(MEDIUM_BUILDUP_DELAY, colorWheel.r, colorWheel.g, colorWheel.b);
  progress += 1;
}

void slowRainbowBuildup() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / BUILDUP_RAINBOW_TIMEFACTOR) % 255);
  buildup(SLOW_BUILDUP_DELAY, colorWheel.r, colorWheel.g, colorWheel.b);
  progress += 1;
}

void veryslowRainbowBuildup() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / BUILDUP_RAINBOW_TIMEFACTOR) % 255);
  buildup(VERYSLOW_BUILDUP_DELAY, colorWheel.r, colorWheel.g, colorWheel.b);
  progress += 1;
}


//////////////////////////
// HardRandom animations

void whiteNoise() {
  noise(MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE); 
}

void redNoise() {
  noise(MAX_INTENSITY_VALUE, 0, 0); 
}

void blueNoise() {
  noise(0, 0, MAX_INTENSITY_VALUE); 
}

void rainbowNoise() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue(int(random()*255));
  noise(colorWheel.r, colorWheel.g, colorWheel.b); 
}


//////////////////////////
// SmoothNoise animations (wave)

void fastWhiteSmoothNoise() {
  smoothNoise(FAST_SINE_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);
}

void fastRedSmoothNoise() {
  smoothNoise(FAST_SINE_VALUE, MAX_INTENSITY_VALUE, 0, 0);
}

void fastBlueSmoothNoise() {
  smoothNoise(FAST_SINE_VALUE, 0, 0, MAX_INTENSITY_VALUE);
}

void fastRainbowSmoothNoise() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);
  smoothNoise(FAST_SINE_VALUE, colorWheel.r, colorWheel.g, colorWheel.b);
}

void slowWhiteSmoothNoise() {
  smoothNoise(SLOW_SINE_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE, MAX_INTENSITY_VALUE);  //To be tested
}

void slowRedSmoothNoise() {
  smoothNoise(SLOW_SINE_VALUE, MAX_INTENSITY_VALUE, 0, 0);
}

void slowBlueSmoothNoise() {
  smoothNoise(SLOW_SINE_VALUE, 0, 0, MAX_INTENSITY_VALUE);
}

void slowRainbowSmoothNoise() {
  CRGB colorWheel;
  colorWheel.CRGB::setHue((millis() / SMOOTHSINE_RAINBOW_TIMEFACTOR) % 255);
  smoothNoise(SLOW_SINE_VALUE, colorWheel.r, colorWheel.g, colorWheel.b);
}

///////////////////////////////////////////////////////////
//////////////////// Generic functions ////////////////////
///////////////////////////////////////////////////////////


void fastFlashDelay() {
  //No delay is alright
  //delay(FAST_FLASH_DELAY);
}

void slowFlashDelay() {
  delay(SLOW_FLASH_DELAY);  
}

void stroboscope(int delayMilli, int r, int g, int b) {
  FastLED.showColor(CRGB(r, g, b));  
  delay(delayMilli);
  FastLED.showColor(CRGB(0, 0, 0));  
  delay(delayMilli);  
}

void smoothSine(float sineSpeed, int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    float intensity = min((0.5 + sin(sineSpeed*(iLed+progress))/2.0), progress/50.0);
    leds[iLed] = CRGB((int) (intensity*r), (int) (intensity*g), (int) (intensity*b));
  }
  FastLED.show();
  delay(SMOOTH_SINE_DELAY);
  progress += 1;
}

void buildup(int delayMilli, int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    if (iLed <= progress) {
      leds[iLed] = CRGB(r, g, b);
    }
    else {
      leds[iLed] = CRGB(0, 0, 0);
    }
  }
  FastLED.show();
  delay(delayMilli);  
}

void smoothNoise(float noiseSpeed, int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    float intensity = min((0.5 + sin(noiseSpeed*(iLed*iLed*iLed+progress))/2), progress/50.0);
    leds[iLed] = CRGB((int) (intensity*r), (int) (intensity*g), (int) (intensity*b));
  }
  FastLED.show();
  progress += 1;
  delay(SMOOTH_SINE_DELAY);
}

void noise(int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    float intensity = random();
    leds[iLed] = CRGB((int) intensity*r, (int) intensity*g, (int) intensity*b);
  }
  FastLED.show();
  delay(NOISE_DELAY);    
}

void upwardWave(int waveLength, int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    leds[iLed] = CRGB(0, 0, 0);
  }
  for (int i = 0; i<waveLength; i++) {
    if ((progress - i) >= 0 && (progress - i) < NUM_LEDS) {
      float intensity = MAX_INTENSITY_VALUE * (1 - (i/(float)(waveLength)));
      leds[progress - i] = CRGB((int) (intensity*r), (int) (intensity*g), (int) (intensity*b));
    }
  }
  FastLED.show();
  progress += 1;
  delay(SINGLE_WAVE_DELAY);
}

void downwardWave(int waveLength, int r, int g, int b) {
  memset(leds, 0,  NUM_LEDS * sizeof(struct CRGB));    
  for(int iLed = 0; iLed < NUM_LEDS; iLed++) {
    leds[iLed] = CRGB(0, 0, 0);
  }
  for (int i = 0; i<waveLength; i++) {
    if ((NUM_LEDS - 1 - (progress - i)) >= 0 && (NUM_LEDS - 1 - (progress - i)) < NUM_LEDS) {
      float intensity = MAX_INTENSITY_VALUE * (1 - (i/(float)(waveLength)));
      leds[NUM_LEDS - 1 - (progress - i)] = CRGB((int) (intensity*r), (int) (intensity*g), (int) (intensity*b));
    }
  }
  FastLED.show();
  progress += 1;
  delay(SINGLE_WAVE_DELAY);
}
