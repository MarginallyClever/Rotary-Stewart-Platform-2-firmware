#ifndef SEGMENT_H
#define SEGMENT_H
//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------


#define MAX_SEGMENTS         (64)


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------


// 32 microstepping with 400 steps per turn cannot exceed 12800.  A signed int is 32767.
struct Axis {
  int step_count;
  int delta;
  int absdelta;
  int dir;
  int over;
};


struct Segment {
  Axis a[NUM_AXIES];
  int steps;
  int steps_left;
  long feed_rate;
};


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern Segment line_segments[MAX_SEGMENTS];
extern volatile int current_segment;
extern volatile int last_segment;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
int get_next_segment(int i);
int get_prev_segment(int i);
void motor_prepare_segment(int n0,int n1,int n2,int n3,int n4,int n5,float new_feed_rate);
void motor_move_segment(Segment &seg);
void motor_move_all_segments();


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
