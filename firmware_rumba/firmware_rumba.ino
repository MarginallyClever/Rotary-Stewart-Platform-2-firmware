//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
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
float fr=0;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?

#ifdef VERBOSE
char *letter="UVWXYZ";
#endif


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for ms values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MAX_FEEDRATE;
  }
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MIN_FEEDRATE;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
}


/**
 * Set the motor position in number of steps
 */
void motor_position(int n0,int n1,int n2,int n3,int n4,int n5) {
  // here is a good place to add sanity tests
  h.arms[0].last_step=n0;
  h.arms[1].last_step=n1;
  h.arms[2].last_step=n2;
  h.arms[3].last_step=n3;
  h.arms[4].last_step=n4;
  h.arms[5].last_step=n5;
  
  // @TODO: not sure this is right...
  Segment &seg = line_segments[last_segment];
  seg.a[0].step_count=n0;
  seg.a[1].step_count=n1;
  seg.a[2].step_count=n2;
  seg.a[3].step_count=n3;
  seg.a[4].step_count=n4;
  seg.a[5].step_count=n5;
}


/**
 * Grips the power on the motors
 **/
void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(h.arms[i].motor_enable_pin,LOW);
  }
}


/**
 * Releases the power on the motors
 **/
void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(h.arms[i].motor_enable_pin,HIGH);
  }
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void hexapod_where() {
  output("X",h.ee.pos.x);
  output("Y",h.ee.pos.y);
  output("Z",h.ee.pos.z);
  output("U",h.ee.r);
  output("V",h.ee.p);
  output("W",h.ee.y);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * print the current motor positions in steps
 */
void motor_where() {
  output("0",h.arms[0].last_step);
  output("1",h.arms[1].last_step);
  output("2",h.arms[2].last_step);
  output("3",h.arms[3].last_step);
  output("4",h.arms[4].last_step);
  output("5",h.arms[5].last_step);
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("StewartPlatform v4-2"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("M17/M18; - enable/disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("F, G00, G01, G04, G28, G90, G91 as described by http://en.wikipedia.org/wiki/G-code"));
  // See hexapod_position() for note about why G92 is removed
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  help();  // say hello

  hexapod_setup();
  motor_enable();
  feedrate(800);  // set default speed
  hexapod_position(0,0,0,0,0,0);
  motor_position(0,0,0,0,0,0);
  hexapod_find_home();
  
  parser_ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  parser_listen();
}


/**
* This file is part of Stewart Platform v2.
*
* Stewart Platform v2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Stewart Platform v2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Stewart Platform v2. If not, see <http://www.gnu.org/licenses/>.
*/
