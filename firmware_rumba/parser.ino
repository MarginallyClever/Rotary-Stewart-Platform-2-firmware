//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
static char buffer[MAX_BUF];  // where we store the message until we get a ';'
static int sofar;  // how much is in the buffer


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr=strchr(ptr,' ')+1;
  }
  return val;
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.print(F("="));
  Serial.println(val);
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void parser_processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0: // move linear
  case  1: // move linear
    feedrate(parsenumber('F',fr));
    hexapod_line( parsenumber('X',(mode_abs?h.ee.pos.x:0)) + (mode_abs?0:h.ee.pos.x),
                  parsenumber('Y',(mode_abs?h.ee.pos.y:0)) + (mode_abs?0:h.ee.pos.y),
                  parsenumber('Z',(mode_abs?h.ee.pos.z:0)) + (mode_abs?0:h.ee.pos.z),
                  parsenumber('U',(mode_abs?h.ee.r:0)) + (mode_abs?0:h.ee.r),
                  parsenumber('V',(mode_abs?h.ee.p:0)) + (mode_abs?0:h.ee.p),
                  parsenumber('W',(mode_abs?h.ee.y:0)) + (mode_abs?0:h.ee.y) );
    break;
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 28:  hexapod_find_home();  break;  
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode

  // See hexapod_position() for note about why G92 is removed
//  case 92:  // set logical position
//    hexapod_position( parsenumber('X',0),
//                      parsenumber('Y',0),
//                      parsenumber('Z',0),
//                      parsenumber('U',0),
//                      parsenumber('V',0),
//                      parsenumber('W',0) );
//    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
  case 100:  help();  break;
  case 114:  hexapod_where();  break;
  default:  break;
  }
}


/**
 * Prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void parser_ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * Listen to the serial port for incoming commands and deal with them
 */
void parser_listen() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF) buffer[sofar++]=c;  // store it
    if(buffer[sofar-1]==';') break;  // entire message received
  }

  if(sofar>0 && buffer[sofar-1]==';') {
    // we got a message and it ends with a semicolon
    buffer[sofar]=0;  // end the buffer so string functions work right
    Serial.print(F("\r\n"));  // echo a return character for humans
    parser_processCommand();  // do something with the command
    parser_ready();
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