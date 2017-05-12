//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
// First time you run the robot send R70 and R71 to initialize the motor angles.
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"
#include "vector3.h"
#include "segment.h"
#include "hexapod.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// speeds
float feed_rate = DEFAULT_FEEDRATE; // how fast the EE moves in cm/s
float acceleration = DEFAULT_ACCELERATION;

// settings
char mode_abs = 1; // absolute mode?

// misc
long robot_uid = 0;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
    finds angle of dy/dx as a value from 0...2PI
    @return the angle
*/
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}


/**
   Delay for the appropriate time. delayMicroseconds() doesn't work for ms values > ~16k.
   @input us how many microseconds to wait
*/
void pause(long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}


/**
   Set the feedrate (speed motors will move)
   @input nfr the new speed in steps/second
*/
float feedrate(float nfr) {
  if (feed_rate == nfr) return nfr; // same as last time?  quit now.

  if (nfr > MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr = MAX_FEEDRATE;
  }
  if (nfr < MIN_FEEDRATE) { // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr = MIN_FEEDRATE;
  }
  feed_rate = nfr;

  return feed_rate;
}


/**
   display helpful information
*/
void help() {
  Serial.print(F("\n\nHELLO WORLD! I AM STEWART PLATFORM V4.2 #"));
  Serial.println(robot_uid, DEC);
  sayVersionNumber();
  Serial.println(F("== http://www.marginallyclever.com/ =="));
  Serial.println(F("I understand the following commands:"));
  Serial.println(F("Nx [A]*y (this is line number x, with checksum y)\n"\
                   "Where A is one of the following:\n"\
                   "G0 [Xx] [Yy] [Zz] (move to XYZ)\n"\
                   "G1 [Xx] [Yy] [Zz] (move to XYZ)\n"\
                   "G2 [Xx] [Yy] [Zz] (move anticlockwise in XY plane to XYZ)\n"\
                   "G3 [Xx] [Yy] [Zz] (move clockwise in XY plane to to XYZ)\n"\
                   "G4 [Pp] [Ss] (wait P seconds and S milliseconds)\n"\
                   "G28 (find home)\n"\
                   "G54-59 [Xx] [Yy] [Zz] (adjust tool offset)\n"\
                   "G90 (absolute movement mode)\n"\
                   "G91 (relative movement mode)\n"\
                   "M17 (disable motors)\n"\
                   "M18 (enable motors)\n"\
                   "M100 (help)\n"\
                   "M110 N* (set line number to *)\n"\
                   "M114 (where)\n"\
                   "UID * (write robot UID * to EEPROM)\n"\
                   "R60 [Aa] [Bb] [Cc] [Dd] [Ed] (set sensors adjustment to new value [abcde])\n"\
                   "R61 (display switch adjustment)\n"\
                   "R70 (write sensor adjustments to memory)\n"\
                   "R71 (reset switch adjustmentles to factory default)\n"));
  // See hexapod_position() for note about why G92 is removed
}


void sayVersionNumber() {
  char versionNumber = loadVersion();

  Serial.print(F("Firmware v"));
  Serial.println(versionNumber, DEC);
}


/**
   First thing this machine does on startup.  Runs only once.
*/
void setup() {
  loadConfig();

  Serial.begin(BAUD);  // open coms

  motor_setup();
  segment_setup();

  hexapod_setup();
  feedrate(DEFAULT_FEEDRATE);  // set default speed

  help();  // say hello
  parser_ready();
}


/**
   After setup() this machine will repeat loop() forever.
*/
void loop() {
#ifdef TEST_STEPPERS
  int i=0;

  for(i=0;i<6;++i) {
    Serial.print("motor ");
    Serial.println(i);
    move_motor(i);
  }

#endif
#ifdef TEST_SWITCHES
  for(int i=0;i<NUM_SWITCHES;++i) {
    int j=robot.arms[i].limit_switch_pin;
    int state=digitalRead(j);
    Serial.print(state?'1':'0');
    Serial.print('\t');
  }
  Serial.println();
  delay(50);
#endif
  
  parser_listen();
}


/**
  This file is part of Stewart Platform v2.

  Stewart Platform v2 is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stewart Platform v2 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Stewart Platform v2. If not, see <http://www.gnu.org/licenses/>.
*/
