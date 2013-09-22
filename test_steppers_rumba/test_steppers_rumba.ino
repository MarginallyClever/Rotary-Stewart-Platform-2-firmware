//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


#define BAUD    57600
#define MAX_BUF 64


struct Arm {
  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int limit_switch_pin;
};


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Arm arms[6];


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  arms[0].motor_step_pin=17;
  arms[0].motor_dir_pin=16;
  arms[0].motor_enable_pin=48;
  arms[0].limit_switch_pin=37;

  arms[1].motor_step_pin=54;
  arms[1].motor_dir_pin=47;
  arms[1].motor_enable_pin=55;
  arms[1].limit_switch_pin=36;

  arms[2].motor_step_pin=57;
  arms[2].motor_dir_pin=56;
  arms[2].motor_enable_pin=62;
  arms[2].limit_switch_pin=35;

  arms[3].motor_step_pin=23;
  arms[3].motor_dir_pin=22;
  arms[3].motor_enable_pin=27;
  arms[3].limit_switch_pin=34;

  arms[4].motor_step_pin=26;
  arms[4].motor_dir_pin=25;
  arms[4].motor_enable_pin=24;
  arms[4].limit_switch_pin=33;

  arms[5].motor_step_pin=29;
  arms[5].motor_dir_pin=28;
  arms[5].motor_enable_pin=39;
  arms[5].limit_switch_pin=32;
  
  int i;
  for(i=0;i<6;++i) {
    pinMode(arms[i].motor_step_pin,OUTPUT);
    pinMode(arms[i].motor_dir_pin,OUTPUT);
    pinMode(arms[i].motor_enable_pin,OUTPUT);
    pinMode(arms[i].limit_switch_pin,INPUT);
    digitalWrite(arms[i].motor_enable_pin,LOW);
    digitalWrite(arms[i].limit_switch_pin,HIGH);
  }
  
  Serial.println("** Ready **");
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  int i=0;

  //for(i=0;i<6;++i) {
    move_motor(i);
  //}
}


void move_motor(int i) {
  int j;
  
  Serial.print("Testing forward ");
  Serial.println(i);

  digitalWrite(arms[i].motor_dir_pin,HIGH);
  for(j=0;j<16*400;++j) {
    digitalWrite(arms[i].motor_step_pin,HIGH);
    digitalWrite(arms[i].motor_step_pin,LOW);
    delayMicroseconds(50);
  }

  Serial.print("Testing backward ");
  Serial.println(i);

  digitalWrite(arms[i].motor_dir_pin,LOW);
  for(j=0;j<16*400;++j) {
    digitalWrite(arms[i].motor_step_pin,HIGH);
    digitalWrite(arms[i].motor_step_pin,LOW);
    delayMicroseconds(50);
  }
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
