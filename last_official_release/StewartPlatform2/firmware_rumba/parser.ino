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
static long last_cmd_time;    // prevent timeouts
long line_number=0;


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


void outputvector(Vector3 &v,char*name) {
  Serial.print(name);
  Serial.print(F("="));
  Serial.print(v.x);
  Serial.print(F(","));
  Serial.print(v.y);
  Serial.print(F(","));
  Serial.println(v.z);
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void parser_processCommand() {
  // blank lines
  if(buffer[0]==';') return;
  
  long cmd;
  
  // is there a line number?
  cmd=parsenumber('N',-1);
  if(cmd!=-1 && buffer[0]=='N') {  // line number must appear first on the line
    if( cmd != line_number ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }
  
    // is there a checksum?
    if(strchr(buffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(buffer[c]!='*') checksum ^= buffer[c++];
      c++; // skip *
      int against = strtod(buffer+c,NULL);
      if( checksum != against ) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      }
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }
    
    line_number++;
  }
  
  cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0: 
  case  1: {  // move in a line
      acceleration = min(max(parsenumber('A',acceleration),1),2000);
      Vector3 offset=robot_get_end_plus_offset();
      robot_line( parsenumber('X',(mode_abs?offset.x:0)) + (mode_abs?0:offset.x),
                  parsenumber('Y',(mode_abs?offset.y:0)) + (mode_abs?0:offset.y),
                  parsenumber('Z',(mode_abs?offset.z:0)) + (mode_abs?0:offset.z),
                  parsenumber('U',(mode_abs?robot.ee.r:0)) + (mode_abs?0:robot.ee.r),
                  parsenumber('V',(mode_abs?robot.ee.p:0)) + (mode_abs?0:robot.ee.p),
                  parsenumber('W',(mode_abs?robot.ee.y:0)) + (mode_abs?0:robot.ee.y),
                  feedrate(parsenumber('F',feed_rate)) );
    break;
  }
  case 2:
  case 3: { // move in an arc
    acceleration = min(max(parsenumber('A',acceleration),1),2000);
    Vector3 offset=robot_get_end_plus_offset();
    robot_arc(parsenumber('I',(mode_abs?offset.x:0)) + (mode_abs?0:offset.x),
              parsenumber('J',(mode_abs?offset.y:0)) + (mode_abs?0:offset.y),
              parsenumber('X',(mode_abs?offset.x:0)) + (mode_abs?0:offset.x),
              parsenumber('Y',(mode_abs?offset.y:0)) + (mode_abs?0:offset.y),
              parsenumber('Z',(mode_abs?offset.z:0)) + (mode_abs?0:offset.z),
              (cmd==2) ? -1 : 1,
              feedrate(parsenumber('F',feed_rate)) );
    break;
  }
  case  4:  {  // dwell
    wait_for_segment_buffer_to_empty();
    pause(parsenumber('S',0) + parsenumber('P',0)*1000);  
    break;
  }
  case 28:  robot_find_home();  break;
  case 54:
  case 55:
  case 56:
  case 57:
  case 58:
  case 59: {  // 54-59 tool offsets
    int tool_id=cmd-54;
    robot_tool_offset(tool_id,parsenumber('X',robot.tool_offset[tool_id].x),
                                parsenumber('Y',robot.tool_offset[tool_id].y),
                                parsenumber('Z',robot.tool_offset[tool_id].z));
    break;
  }
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode

  // See hexapod_position() for note about why G92 is removed
  case 92: { // set logical position
    Vector3 offset = robot_get_end_plus_offset();
    robot_position( parsenumber('X',offset.x),
                    parsenumber('Y',offset.y),
                    parsenumber('Z',offset.z),
                    parsenumber('U',0),
                    parsenumber('V',0),
                    parsenumber('W',0) );
    }
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 6:  robot_tool_change(parsenumber('T',robot.current_tool));  break;
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
  case 100:  help();  break;
  case 110:  line_number = parsenumber('N',line_number);  break;
  case 114:  robot_where();  break;
  default:  break;
  }
}


/**
 * Prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void parser_ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
  last_cmd_time = millis();
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
    if(c=='\n') {
      buffer[sofar]=0;  // end the buffer so string functions work right
      //Serial.print(F("\r\n"));  // echo a return character for humans
      parser_processCommand();  // do something with the command

#ifdef ONE_COMMAND_AT_A_TIME
      wait_for_segment_buffer_to_empty();
#endif

      parser_ready();
      return;
    }
  }
  
  // The PC will wait forever for the ready signal.
  // if Arduino hasn't received a new instruction in a while, send ready() again
  // just in case USB garbled ready and each half is waiting on the other.
  if( !segment_buffer_full() && (millis() - last_cmd_time) > TIMEOUT_OK ) {
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
