#ifndef HEXAPOD_H
#define HEXAPOD_H
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


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
struct EndEffector  {
  Vector3 up;
  Vector3 left;
  Vector3 forward;
  Vector3 pos;
  Vector3 relative;
  float r,p,y;  // roll, pitch, yaw
};


struct Arm {
  Vector3 shoulder;
  Vector3 elbow;
  Vector3 shoulder_to_elbow;
  Vector3 wrist;

  float angle;
  int last_step;
  int new_step;

  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int motor_scale;  // 1 or -1

  char limit_switch_pin;
  int limit_switch_state;
};


struct Hexapod {
  Vector3 base;
  Arm arms[NUM_AXIES];
  EndEffector ee;
 
  Vector3 v;  // max XYZ vel
  Vector3 destination;  // target XYZ position
};


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern Hexapod h;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
void hexapod_setup();
void hexapod_update_ik(Vector3 &mov,Vector3 &rpy);

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
#endif
