#include <MIDI.h>

//Parametres pour le JMP1 : correspond au numero de preset que tu souhaites affecter
const int MIDI_CHANNEL_JMP1 = 2;          //MIDI Channel 2
const int JMP1_CLEAN_1   = 0;
const int JMP1_CLEAN_2   = 1;
const int JMP1_CLEAN_3   = 2;
const int JMP1_CRUNCH_1  = 3;
const int JMP1_CRUNCH_2  = 4;
const int JMP1_CRUNCH_3  = 5;
const int JMP1_DISTO_1   = 6;
const int JMP1_DISTO_2   = 7;
const int JMP1_DISTO_3   = 8;
const int JMP1_BOOST_1   = 9;
const int JMP1_BOOST_2   = 10;
const int JMP1_BOOST_3   = 11;
const int JMP1_SPECIAL_1 = 12;
const int JMP1_SPECIAL_2 = 13;
const int JMP1_SPECIAL_3 = 14;


//Parametres pour le Line6 : correspond au numero de preset que tu souhaites affecter
//Note pour le Line6 / 0->preset 1A bankA, 63->preset 16D bankA, 64->preset 1A bankB, etc.
const int MIDI_CHANNEL_LINE6 = 3;          //MIDI Channel 4
const int POD_HDPRO_NOEFFECT        = 0;
const int POD_HDPRO_WHAMMY          = 1;
const int POD_HDPRO_PHASER          = 2;
const int POD_HDPRO_HIPASSFILTER    = 3;
const int POD_HDPRO_LOWPASSFILTER   = 4;
const int POD_HDPRO_BANDPASSFILTER  = 5;
const int POD_HDPRO_SYNTH           = 6;
const int POD_HDPRO_OCTAVER         = 7;
const int POD_HDPRO_DISTO           = 8;
const int POD_HDPRO_SON_CHELOU      = 9;
const int POD_HDPRO_PRESET_1A_BANK2 = 64;

//Parametres pour le Voicelive : correspond au numero de preset que tu souhaites affecter
const int MIDI_CHANNEL_VOICELIVE = 4;      //MIDI Channel 5
const int VOICELIVE_NOEFFECT = 0;
const int VOICELIVE_FILTER   = 1;
const int VOICELIVE_HARMONY  = 2;
const int VOICELIVE_OCTAVER  = 3;

//Preset list : Un preset (output MIDI Program Change) pour chaque input MIDI Program Change
//Note : seules les trois dernieres cases du tableau sont utilisees, la premiere est juste la pour que ce soit plus lisible
//Au passage, la premiere case du tableau commence a 0, mais tu peux la faire commencer a 1 si c'est plus lisible pour toi (en fonction de la numerotation des boutons sur le Rocktron)

////////////////////////////////////////////////////////
////////// CONFIG LINE6+JMP1+VOICELIVE PLAYME //////////
////////////////////////////////////////////////////////

//// PlayMe ///

int PlayMePresets[128][4] = {  {1   , JMP1_SPECIAL_1  , POD_HDPRO_HIPASSFILTER      , VOICELIVE_HARMONY},
                               {2   , JMP1_CLEAN_3    , POD_HDPRO_PHASER            , VOICELIVE_NOEFFECT},
                               {3   , JMP1_CRUNCH_1   , POD_HDPRO_HIPASSFILTER      , VOICELIVE_NOEFFECT},
                               {4   , JMP1_CRUNCH_2   , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {5   , JMP1_CRUNCH_3   , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {6   , JMP1_DISTO_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {7   , JMP1_DISTO_2    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {8   , JMP1_DISTO_3    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {9   , JMP1_BOOST_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {10  , JMP1_BOOST_2    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {11  , JMP1_BOOST_3    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {12  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {13  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {14  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {15  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {16  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {17  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {18  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {19  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {20  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {21  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {22  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {23  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {24  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {25  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {26  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {27  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {28  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {29  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {30  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {31  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {32  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {33  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {34  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {35  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {36  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {37  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {38  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {39  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {40  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {41  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {42  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {43  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {44  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {45  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {46  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {47  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {48  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {49  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {50  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {51  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {52  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {53  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {54  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {55  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {56  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {57  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {58  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {59  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {60  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {61  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {62  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {63  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {64  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {65  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {66  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {67  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {68  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {69  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {70  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {71  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {72  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {73  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {74  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {75  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {76  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {77  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {78  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {79  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {80  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {81  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {82  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {83  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {84  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {85  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {86  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {87  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {88  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {89  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {90  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {91  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {92  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {93  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {94  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {95  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {96  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {97  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {98  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {99  , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {100 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {101 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {102 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {103 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {104 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {105 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {106 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {107 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {108 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {109 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {110 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {111 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {112 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {113 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {114 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {115 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {116 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {117 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {118 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {119 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {120 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {121 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {122 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {123 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {124 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {125 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {126 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {127 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT},
                               {128 , JMP1_CLEAN_1    , POD_HDPRO_NOEFFECT          , VOICELIVE_NOEFFECT}};
                               
                               


//Utile pour voir que le systeme est en fonctionnement : allume une LED RGB et change sa couleur
const int redPin =  12;
const int greenPin =  15;
const int bluePin =  14;
int colorWheelVal = 0;
//Ralentit le cycle des couleurs
int colorSpeedFactor = 100;

struct ColorTable {
  int r;
  int g;
  int b;
};

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.begin(9600); // USB is always 12 Mbit/sec
  //Configuration LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  int type, note, velocity, channel;
  if (MIDI.read()) {           //Incoming MIDI message ?
    byte type = MIDI.getType();
    int note = MIDI.getData1();
    int velocity = MIDI.getData2();
    int channel = MIDI.getChannel();
    switch (type) {
      case NoteOn:
        MIDI.sendNoteOn(note, velocity, channel);
        break;
      case NoteOff:
        MIDI.sendNoteOff(note, velocity, channel);
        break;
      case PitchBend:
        MIDI.sendPitchBend(note, channel);  //sendPitchBend(int PitchValue,byte Channel); //Pitchvalue is probably contained in data1, stored in note
        break;
      case ControlChange:
        MIDI.sendControlChange(note, velocity, MIDI_CHANNEL_LINE6);    //A tester : bypass du control change pour les autres devices
        break;
      case ProgramChange:
        specificProgramChangeLine6(note, velocity);
        specificProgramChangeJMP1(note, velocity);
        specificProgramChangeVoiceLive(note, velocity);
        break;
      default:
        break;
    }
  }
  
  //Allume la LED
  lightUpTheSky();
}


void specificProgramChangeLine6(int note, int velocity) {
    int bank, newnote;
    int tempnote = PlayMePresets[note][2];
    if (tempnote < 63) {
      bank = 0;
      newnote = tempnote;
    }
    else if (tempnote < 127) {
      bank = 1;
      newnote = tempnote - 64;
    }
    else {
      bank = 0;
      newnote = 0;
    }
    MIDI.sendControlChange(0, 0, MIDI_CHANNEL_LINE6);                 //Bank change : MSB
    MIDI.sendControlChange(32, bank, MIDI_CHANNEL_LINE6);             //Bank change : LSB
    MIDI.sendProgramChange(newnote, MIDI_CHANNEL_LINE6);              //Change the program    //MIDI.h reference : sendProgramChange(byte ProgramNumber,byte Channel); ProgramNumber is probably contained in data1
}


void specificProgramChangeJMP1(int note, int velocity) {
    int ampPreset = 0;
    if (note >= 0 && note <= 127) {
      ampPreset = PlayMePresets[note][1];
    }
    MIDI.sendProgramChange(ampPreset, MIDI_CHANNEL_JMP1);    //Change the program
}

void specificProgramChangeVoiceLive(int note, int velocity) {
    int voicePreset = 0;
    if (note >= 0 && note <= 127) {
      voicePreset = PlayMePresets[note][3];
    }
    MIDI.sendProgramChange(voicePreset, MIDI_CHANNEL_VOICELIVE);
}

void lightUpTheSky() {
  
  struct ColorTable colorValues = getColorWheel(colorWheelVal);
  analogWrite(redPin, colorValues.r);
  analogWrite(greenPin, colorValues.g);
  analogWrite(bluePin, colorValues.b);
  colorWheelVal = (colorWheelVal + 1) % (255 * colorSpeedFactor);
}

struct ColorTable getColorWheel(int WheelPos) {
  struct ColorTable colors;
  WheelPos = int(WheelPos/colorSpeedFactor);
  if (WheelPos < 85) {
    colors.r = WheelPos * 3;
    colors.g = 255 - WheelPos * 3;
    colors.b = 0;
  } 
  else if (WheelPos < 170) {
    WheelPos -= 85;
    colors.r = 255 - WheelPos * 3;
    colors.g = 0;
    colors.b = WheelPos * 3;
  } 
  else {
    WheelPos -= 170; 
    colors.r = 0;
    colors.g = WheelPos * 3;
    colors.b = 255 - WheelPos * 3;
  }

  return colors;
}
