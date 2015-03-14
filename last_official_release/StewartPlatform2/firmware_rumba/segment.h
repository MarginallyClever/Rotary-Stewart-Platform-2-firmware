#ifndef SEGMENT_H
#define SEGMENT_H
//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.




//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------


// 32 microstepping with 400 steps per turn cannot exceed 12800.  A signed int is 32767.
struct Axis {
  int step_count;
  int absdelta;
  int delta;
  int dir;
  float delta_normalized;
};


struct Segment {
  Axis a[NUM_AXIES];
  int steps_total;
  int steps_taken;
  int accel_until;
  int decel_after;
  float feed_rate_max;
  float feed_rate_start;
  float feed_rate_start_max;
  float feed_rate_end;
  char nominal_length_flag;
  char recalculate_flag;
  char busy;
};



//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern Segment line_segments[MAX_SEGMENTS];
extern Segment *working_seg;
extern volatile int current_segment;
extern volatile int last_segment   ;



//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
int get_next_segment(int i);
int get_prev_segment(int i);
void motor_prepare_segment(int n0,int n1,int n2,int n3,int n4,int n5,float new_feed_rate);
void motor_move_segment(Segment &seg);
void motor_move_all_segments();



// for reasons I don't understand... if i put this method in the .ino file i get compile errors.
// so I put it here, which forces the externs.
FORCE_INLINE Segment *segment_get_working() {
  if(current_segment == last_segment ) return NULL;
  working_seg = &line_segments[current_segment];
  return working_seg;
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
#endif
