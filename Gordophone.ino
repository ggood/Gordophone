/*

Prototype sketch for a trombone-like MIDI controller running on the Teensy 2.0 hardware.

Hardware:

- An set of four switches used to select an overtone. We use "chording" to allow
  the 4 switches to select overtones. I'm not sure what the most natural method
  of chording is, but let's try the following:
  
  Switch
  3210      Overtone
  0000      OT_1
  0001      OT_2
  0011      OT_3
  0111      OT_4
  1111      OT_5
  1110      OT_7
  1100      OT_8
  1000      OT_8
  
  Switches 0-3 are wired to pull Arduino digital input pins 2-5 low when
  pressed.
  
- A "slide". Currently, this produces pitch bend information, and is implemented
  with a 500mm SpectraSymbol SoftPot linear resistance strip.

- A volume controller, implemented with a FreeScale pressure sensor. The player
  blows into a tube that goes to a "T" - one leg goes to the pressure sensor, and
  the other is open (a "dump tube") so that the player can put air through the
  instrument.

July 3, 2012
Gordon Good (velo27 <at> yahoo <dot> com)

*/

#include <Bounce.h> 

// If DEBUG == true, then the sketch will print to the serial port what
// it would send on the MIDI bus.
const boolean DEBUG = false;
//const boolean DEBUG = true;

const int BREATH_PIN = 0; // Breath sensor on analog pin 0
const int SLIDE_LPOT_PIN = 1; // Slide sensor on analog pin 1

const int OT_SW_0_PIN = 2; // Overtone switch 0
const int OT_SW_1_PIN = 3; // Overtone switch 1
const int OT_SW_2_PIN = 4; // Overtone switch 2
const int OT_SW_3_PIN = 5; // Overtone switch 3

const int SLIDE_QUANT_TOGGLE_PIN = 6; // Pin that toggles slide quantization on/off
const int SLIDE_LED_PIN = 13; //Pin that drives LED that shows slide quantization

const int PANIC_PIN = 7; // MIDI all notes off momentary switch on digital I/O 4

// The overtone series this instrument will produce
const int FUNDAMENTAL = 36;  // MIDI note value of our fundamental
const int OT_1 = 48; // First overtone (B flat)
const int OT_2 = 55; // Second overtone (F)
const int OT_3 = 60; // Third overtone (B flat)
const int OT_4 = 64; // Fourth overtone (D)
const int OT_5 = 67; // Fifth overtone (F)
const int OT_6 = 70; // Sixth overtone (A flat - not in tune - need to tweak pitch bend)
const int OT_7 = 72; // Seventh overtone (B flat)
const int OT_8 = 74; // Eighth overtone (C)
const int OT_9 = 76; // Ninth overtone (D)
const int OT_NONE = -1; // No overtone key pressed (not possible with ribbon)

// All overtones for this instrument
const int overtones[10] =    {FUNDAMENTAL, OT_1, OT_2, OT_3, OT_4, OT_5, OT_6, OT_7, OT_8, OT_9};
// Switch values for given overtones. 0xff means that overtone can't be selected.
const int overtone_sw_values[10] = {0xff, 0x00, 0x01, 0x03, 0x07, 0x0f, 0x0e, 0x0c, 0x08, 0xff};

const int MIDI_VOLUME_CC = 7; // The controller number for MIDI volume data
const int MIDI_BREATH_CC = 2; // The controller number for MIDI breath controller data

long ccSendTime = 0;  // Last time we sent continuous data (volume, pb);
const int MIN_CC_INTERVAL = 10; // Send CC data no more often than this (in milliseconds);
const int PB_SEND_THRESHOLD = 10; // Only send pitch bend if it's this much different than the current value
const int VOLUME_SEND_THRESHOLD = 1; // Only send volume change if it's this much differnt that the current value
const int NOTE_ON_VOLUME_THRESHOLD = 56; // Raw sensor value required to turn on a note
const int VOLUME_MAX_VALUE = 500; // Maximum value from the breath sensor.

// If a value larger than this is read from a SoftPot, treat it as if the player is not touching it.
// Note: for some reason, the two SoftPots interact, e.g. just actuating the slide pot gives me
// no-touch values all above 1000, but when also touching the overtone pot, the values can go
// as low as 999. I suspect I may be taxing the 5v supply line.
const int LPOT_NO_TOUCH_VALUE = 1010;  
const int LPOT_SLIDE_POS_1 = 945; // Value at 1st position
const int LPOT_SLIDE_POS_7 = 30;  // Value at 7th position
const int MAX_PITCH_BEND_DOWN = 0; // Pitch bend value for 7th position
const int PITCH_BEND_NEUTRAL = 16383 / 2; // Neutral pitch bend value

int currentNote = -1; // The MIDI note currently sounding
int currentPitchBend = PITCH_BEND_NEUTRAL; // The current pitch bend
int currentVolume = 0; // The current volume
int slide_quant_mode = 0; // The current slide quantization mode. 0 = disabled, 1 = enabled

// Debounce debouncer = Debounce( 20 , SLIDE_QUANT_TOGGLE_PIN);
Bounce slideQuantToggleButton = Bounce(SLIDE_QUANT_TOGGLE_PIN, 20);

void setup() {
  pinMode(OT_SW_0_PIN, INPUT_PULLUP);
  pinMode(OT_SW_1_PIN, INPUT_PULLUP);
  pinMode(OT_SW_2_PIN, INPUT_PULLUP);
  pinMode(OT_SW_3_PIN, INPUT_PULLUP);
  pinMode(SLIDE_QUANT_TOGGLE_PIN, INPUT_PULLUP);
  pinMode(PANIC_PIN, INPUT_PULLUP);
  enableDigitalOutput(SLIDE_LED_PIN);
  enableAnalogInput(BREATH_PIN, false);
  enableAnalogInput(SLIDE_LPOT_PIN, true);
  
  if (DEBUG) {
    Serial.begin(9600);
  }
}


/**
 * Enable a pin for analog input, and set its internal pullup.
 */
void enableAnalogInput(int pin, boolean enablePullup) {
  pinMode(pin, INPUT);
  digitalWrite(pin + 14, enablePullup ? HIGH : LOW);
}


/**
 * Enable a pin for digital input, and set its internal pullup.
 */
void enableDigitalInput(int pin, boolean enablePullup) {
  pinMode(pin, INPUT);
  digitalWrite(pin, enablePullup ? HIGH : LOW);
}


/**
 * Enable a pin for digital output.
 */
void enableDigitalOutput(int pin) {
  pinMode(pin, OUTPUT);
}


/**
 * Read the slide pot and return a pitch bend value. The values
 * returned are all bends down from the base pitch being played,
 * and are in the range 8192 (no bend) to 0 (maximum bend down).
 * This means that the synth patch needs to be adjusted to provide
 * a maximum pitch bend of seven semitones, if you want it to
 * behave like a trombone.
 */
 int getPitchBendFromLinearPot() {

  // Toggle slide quantization mode, if requested
  slideQuantToggleButton.update();
  if (slideQuantToggleButton.fallingEdge()) {
    slide_quant_mode = ~slide_quant_mode;
    digitalWrite(SLIDE_LED_PIN, slide_quant_mode);
  }
  
  // Get the raw value from the linear pot
  int slideVal = analogRead(SLIDE_LPOT_PIN);
  
  if (slideVal > LPOT_NO_TOUCH_VALUE) {
    return -1;
  } else {
    // Coerce out-of-range values (e.g. beyond the slide stops)
    int constrainedVal = slideVal;
    constrainedVal = constrainedVal < LPOT_SLIDE_POS_7 ? LPOT_SLIDE_POS_7 : constrainedVal;
    constrainedVal = constrainedVal > LPOT_SLIDE_POS_1 ? LPOT_SLIDE_POS_1 : constrainedVal;
    
   int  pbVal = map(constrainedVal, LPOT_SLIDE_POS_1, LPOT_SLIDE_POS_7, PITCH_BEND_NEUTRAL, MAX_PITCH_BEND_DOWN);
   if (pbVal < 0) pbVal = 0;
    
    // Quantize slide position, if requested
    if (slide_quant_mode) {
      pbVal = quantizeSlide(pbVal);
    }
    return pbVal;
  }
}


/*
 * Quantize the slide so that there are only seven possible values.
 * Each "position" is one position wide, centered on the actual
 * slide position value, except for 1st and 7th positions, which
 * are only half a position wide:
 *
 * 1    2    3    4    5    6    7
 * ^^^                           
 *    ^^^^^                        
 *         ^^^^^                  
 *              ^^^^^            
 *                   ^^^^^       
 *                        ^^^^^  
 *                             ^^^
 */
int quantizeSlide(int val) {
  if (val >= 0 && val <= 683) return 0;
  if (val >= 684 && val <= 2048) return 1365;
  if (val >= 2049 && val <= 3413) return 2731;
  if (val >= 3414 && val <= 4779) return 4096;
  if (val >= 4780 && val <= 6144) return 5461;
  if (val >= 6145 && val <= 7509) return 6827;
  if (val >= 7510 && val <= 8192) return 8191;
  return 0;
}


/*
 * Read the slide and return a pitch bend value in the range from
 * 8192 (1st position) to 0 (7th position)
 */
int getPitchBend() {
  return getPitchBendFromLinearPot();
}


/**
 * Read the overtone switches and return the appropriate overtone.
 * If an invalid key combination is found, return -1. Note that
 * we invert the values from digitalRead, since these switches 
 * pull to ground, so switch enabled = digital 0.
 */
int getOvertoneFromOvertoneSwitches() {
  unsigned char val = !digitalRead(OT_SW_3_PIN);
  val = val << 1 | !digitalRead(OT_SW_2_PIN);
  val = val << 1 | !digitalRead(OT_SW_1_PIN);
  val = val << 1 | !digitalRead(OT_SW_0_PIN);
  // now select the appropriate overtone
  for (int i = 0; i < sizeof(overtone_sw_values); i++)  {
    if (val == overtone_sw_values[i]) {
      return i;
    }
  }
  return -1;
}


int getMIDINote() {
  int ot = getOvertoneFromOvertoneSwitches();
  if (-1 == ot) {
    return currentNote;
  } else {
    return overtones[ot];
  }
}
  
/**
 * Read the breath sensor and map it to a volume level. For now,
 * this maps to the range 0 - 127 so we can generate MIDI
 * continuous controller information.
 */
int getVolumeFromBreathSensor() {
  int volRawVal = analogRead(BREATH_PIN);
  if (volRawVal < NOTE_ON_VOLUME_THRESHOLD) {
    return 0;
  } else {
    return map(constrain(volRawVal, NOTE_ON_VOLUME_THRESHOLD, VOLUME_MAX_VALUE), NOTE_ON_VOLUME_THRESHOLD, VOLUME_MAX_VALUE, 0, 127);
  }
}

int getVolume() {
  return getVolumeFromBreathSensor();
}

void sendNoteOn(int note, int vel, byte chan, boolean debug) {
  if (debug) {
    Serial.print("ON ");
    Serial.println(note);
  } else {
    //MidiUart.sendNoteOn(chan, note, vel);
    usbMIDI.sendNoteOn(note, vel, chan);
  }
}

void sendNoteOff(int note, int vel, byte chan, boolean debug) {
  if (debug) {
    Serial.print("OFF ");
    Serial.println(note);
  } else {
    //MidiUart.sendNoteOff(chan, note, vel);
    usbMIDI.sendNoteOff(note, vel, chan);
  }
}


void sendPitchBend(int pitchBend, byte chan, boolean debug) {
  if (-1 != pitchBend) {
    if (abs(currentPitchBend - pitchBend) > PB_SEND_THRESHOLD) {
      currentPitchBend = pitchBend;
      if (debug) {
        Serial.print("BEND ");
        Serial.println(pitchBend);
      } else {
        //MidiUart.sendPitchBend(pitchBend);
        usbMIDI.sendPitchBend(pitchBend, chan);
      }
    }
  }
}

void sendVolume(int volume, byte chan, boolean debug) {
  if (abs(currentVolume - volume) > VOLUME_SEND_THRESHOLD) {
    currentVolume = volume;
    if (debug) {
      Serial.print("VOL ");
      Serial.println(volume);
    } else {
      //midi.sendControlChange(chan, MIDI_VOLUME_CC, volume);
      //MidiUart.sendCC(chan, MIDI_VOLUME_CC, 100 );
      usbMIDI.sendControlChange(MIDI_VOLUME_CC, volume, chan);
    }
  }
}

void sendBreathController(int volume, byte chan, boolean debug) {
  if (abs(currentVolume - volume) > VOLUME_SEND_THRESHOLD) {
    if (debug) {
      Serial.print("BC ");
      Serial.println(volume);
    } else {
      //MidiUart.sendCC(chan, MIDI_BREATH_CC, volume );
      usbMIDI.sendControlChange(MIDI_BREATH_CC, volume, chan);
    }
  }
}

void allNotesOff() {
  for (int i = 0; i < 128; i++) {
    sendNoteOff(i, 0, 1, DEBUG);
  }
}

void loop() {
  
  if (digitalRead(PANIC_PIN) == 0) {
    allNotesOff();
  }
  
  int pb = getPitchBend();
  int note = getMIDINote();
  int volume = getVolume();
  
  if ((-1 != currentNote) && (0 == volume)) {
    // Breath stopped, so send a note off
    sendNoteOff(currentNote, 0, 1, DEBUG);
    currentNote = -1;
  } else if ((-1 == currentNote) && (0 != volume) && (-1 != note)) {
    // No note was playing, and we have breath and a valid overtone, so send a note on.
    // Be sure to send any updated pitch bend first, though, in case the slide moved.
    sendPitchBend(pb, 1, DEBUG);
    sendNoteOn(note, 127, 1, DEBUG);
    currentNote = note;
  } else if ((-1 != currentNote) && (note != currentNote)) {
    // A note was playing, but the player has moved to a different note.
    // Turn off the old note and turn on the new one.
    sendNoteOff(currentNote, 0, 1, DEBUG);
    sendPitchBend(pb, 1, DEBUG);
    sendBreathController(volume, 1, DEBUG);
    sendNoteOn(note, 127, 1, DEBUG);
    currentNote = note;
  } else if (-1 != currentNote) {
    // Send updated breath controller and pitch bend values.
    if (millis() > ccSendTime + MIN_CC_INTERVAL) {
      sendPitchBend(pb, 1, DEBUG);
      sendBreathController(volume, 1, DEBUG);
      ccSendTime = millis();
    }
  }
  delay(50);
}

