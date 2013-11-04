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
  output("X",h.ee.pos.x);
  output("Y",h.ee.pos.y);
  output("Z",h.ee.pos.z);
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
 * Set the clock 1 timer frequency.
 * @input desired_freq_hz the desired frequency
 */
void timer_set_frequency(long desired_freq_hz) {
  // Source: https://github.com/MarginallyClever/ArduinoTimerInterrupt
  // Different clock sources can be selected for each timer independently. 
  // To calculate the timer frequency (for example 2Hz using timer1) you will need:
  
  //  CPU frequency 16Mhz for Arduino
  //  maximum timer counter value (256 for 8bit, 65536 for 16bit timer)
  int prescaler_index=-1;
  int prescalers[] = {1,8,64,256,1024};
  long counter_value;
  do {
    ++prescaler_index;
    //  Divide CPU frequency through the choosen prescaler (16000000 / 256 = 62500)
    counter_value = CLOCK_FREQ / prescalers[prescaler_index];
    //  Divide result through the desired frequency (62500 / 2Hz = 31250)
    counter_value /= desired_freq_hz;
    //  Verify counter_value < maximum timer. if fail, choose bigger prescaler.
  } while(counter_value > MAX_COUNTER && prescaler_index<4);
  
  if( prescaler_index>=5 ) {
    Serial.println(F("Timer could not be set: Desired frequency out of bounds."));
    return;
  }

  // disable global interrupts
  noInterrupts();
  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set entire TCCR1B register to 0
  TCCR1B = 0;
  // set the overflow clock to 0
  TCNT1  = 0;
  // set compare match register to desired timer count
  OCR1A = counter_value;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for prescaler
  TCCR1B |= ( (( prescaler_index&0x1 )   ) << CS10);
  TCCR1B |= ( (( prescaler_index&0x2 )>>1) << CS11);
  TCCR1B |= ( (( prescaler_index&0x4 )>>2) << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts
  interrupts();
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
