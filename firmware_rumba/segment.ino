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
#include "segment.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// A ring buffer of line segments.
// @TODO: process the line segments in another thread
// @TODO: optimize speed between line segments
Segment line_segments[MAX_SEGMENTS];
int current_segment=0;
int last_segment=0;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
int get_next_segment(int i) {
  return ( i + 1 ) % MAX_SEGMENTS;
}


int get_prev_segment(int i) {
  return ( i + MAX_SEGMENTS - 1 ) % MAX_SEGMENTS;
}


/**
 * Add a segment to the line buffer if there is room.
 */
void motor_prepare_segment(int n0,int n1,int n2,int n3,int n4,int n5) {
  int next_segment = get_next_segment(last_segment);
  if( next_segment == current_segment ) {
    Serial.println(F("Segment buffer overflow."));
    return;
  }
  
  Segment &old_seg = line_segments[last_segment];
  Segment &new_seg = line_segments[next_segment];
  
  new_seg.a[0].step_count = n0;
  new_seg.a[1].step_count = n1;
  new_seg.a[2].step_count = n2;
  new_seg.a[3].step_count = n3;
  new_seg.a[4].step_count = n4;
  new_seg.a[5].step_count = n5;
  
  new_seg.a[0].delta = n0 - old_seg.a[0].step_count;
  new_seg.a[1].delta = n1 - old_seg.a[1].step_count;
  new_seg.a[2].delta = n2 - old_seg.a[2].step_count;
  new_seg.a[3].delta = n3 - old_seg.a[3].step_count;
  new_seg.a[4].delta = n4 - old_seg.a[4].step_count;
  new_seg.a[5].delta = n5 - old_seg.a[5].step_count;

  new_seg.steps=0;

  int i,j;
  for(i=0;i<NUM_AXIES;++i) {
    new_seg.a[i].over=0;
    new_seg.a[i].dir = (new_seg.a[i].delta > 0 ? 1:-1);
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    if( new_seg.steps < new_seg.a[i].absdelta ) {
      new_seg.steps = new_seg.a[i].absdelta;
    }
  }

#ifdef VERBOSE
  Serial.print(F("Added segment "));
  Serial.println(last_segment);
#endif
  last_segment=get_next_segment(last_segment);
}

 
/**
 * Uses bresenham's line algorithm to move both motors.  Optimized to faster than onestep()
 * @input seg the line segment to move
 **/
void motor_move_segment(Segment &seg) {
  int i,j;
  
  // set the directions once per segment
  for(j=0;j<NUM_AXIES;++j) {
    digitalWrite( h.arms[j].motor_dir_pin, (seg.a[j].dir * h.arms[j].motor_scale > 0 )? LOW:HIGH );
  }
  
  for(i=0;i<seg.steps;++i) {
    for(j=0;j<NUM_AXIES;++j) {
      Axis &a = seg.a[j];
      
      a.over += a.absdelta;
      if(a.over >= seg.steps) {
        a.over -= seg.steps;
        
        digitalWrite(h.arms[j].motor_step_pin,HIGH);
        digitalWrite(h.arms[j].motor_step_pin,LOW);
      }
    }
    // @TODO: change to seg.step_delay?
    pause(step_delay);
  }
}


/**
 * process all line segments in the ring buffer
 */
void motor_move_all_segments() {
  while(current_segment!=last_segment) {
    current_segment = get_next_segment(current_segment);
    
#ifdef VERBOSE
    Serial.print(F("line_segments["));
    Serial.print(current_segment);
    Serial.print(F("].a[0].delta="));
    Serial.println(line_segments[current_segment].a[0].delta);
#endif    
    motor_move_segment(line_segments[current_segment]);
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
