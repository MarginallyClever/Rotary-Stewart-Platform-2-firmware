//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


int old_state[6];


void setup() {
  Serial.begin(57600);
  for(int i=0;i<6;++i) {
    int j=37-i;
    pinMode(j,INPUT);
    digitalWrite(j,HIGH);
    old_state[i]=-1;
  }
}


void loop() {
  for(int i=0;i<6;++i) {
    int j=37-i;
    int state=digitalRead(j);
    if(old_state[i] != state) {
      old_state[i] = state;
      Serial.print(j);
      Serial.print('=');
      Serial.println(state);
    }
  }
  delay(50);
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
