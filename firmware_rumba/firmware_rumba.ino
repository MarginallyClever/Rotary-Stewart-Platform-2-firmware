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
float feed_rate=0;  // human version

// settings
char mode_abs=1;  // absolute mode?

#ifdef VERBOSE
char *letter="UVWXYZ";
#endif


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
  * finds angle of dy/dx as a value from 0...2PI
  * @return the angle
  */
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


/**
 * Delay for the appropriate time. delayMicroseconds() doesn't work for ms values > ~16k.
 * @input us how many microseconds to wait
 */
void pause(long us) {
  delay(us/1000);
  delayMicroseconds(us%1000);
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
float feedrate(float nfr) {
  if(feed_rate==nfr) return nfr;  // same as last time?  quit now.

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
  feed_rate=nfr;
  
  return feed_rate;
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
  Vector3 offset = hexapod_get_end_plus_offset;
  output("X",offset.x);
  output("Y",offset.y);
  output("Z",offset.z);
  output("U",h.ee.r);
  output("V",h.ee.p);
  output("W",h.ee.y);
  output("F",feed_rate);
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

  pinMode(13,OUTPUT);

  hexapod_setup();
  motor_enable();
  feedrate(200);  // set default speed
  hexapod_position(0,0,0,0,0,0);
  motor_position(0,0,0,0,0,0);
  //hexapod_find_home();
  
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
