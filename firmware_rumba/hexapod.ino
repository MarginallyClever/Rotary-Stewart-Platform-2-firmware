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
#include "hexapod.h"
#include "segment.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Hexapod robot;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * Build a virtual model of the hexapod at home position.
 */
void hexapod_setup() {
  Vector3 zero(0,0,0);
  
  hexapod_update_endeffector(zero,zero);
  hexapod_build_shoulders();
  hexapod_update_wrists();

  // find the starting height of the end effector at home position
  // @TODO: project wrist-on-bicep to get more accurate distance
  Vector3 el=robot.arms[0].elbow;
  Vector3 wr=robot.arms[0].wrist;
  float aa=(el.y-wr.y);
  float cc=FOREARM_LENGTH;
  float bb=sqrt((cc*cc)-(aa*aa));
  aa=el.x-wr.x;
  cc=bb;
  bb=sqrt((cc*cc)-(aa*aa));
  robot.ee.relative.Set(0,0,bb+B2S_Z-T2W_Z);

  update_ik(zero,zero);
  robot_position(0,0,0,0,0,0);
  hexapod_loadHomeAngles();
}

void hexapod_setupAnglesFirstTime() {
  robot.steps_to_zero[0]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE1;
  robot.steps_to_zero[1]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE2;
  robot.steps_to_zero[2]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE3;
  robot.steps_to_zero[3]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE4;
  robot.steps_to_zero[4]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE5;
  robot.steps_to_zero[5]=MICROSTEP_PER_DEGREE * 90.00 - SWITCH_ANGLE6;
}


void hexapod_loadHomeAngles() {
  robot.steps_to_zero[0] = EEPROM_readLong(ADDR_ANGLE1);
  robot.steps_to_zero[1] = EEPROM_readLong(ADDR_ANGLE2);
  robot.steps_to_zero[2] = EEPROM_readLong(ADDR_ANGLE3);
  robot.steps_to_zero[3] = EEPROM_readLong(ADDR_ANGLE4);
  robot.steps_to_zero[4] = EEPROM_readLong(ADDR_ANGLE5);
  robot.steps_to_zero[5] = EEPROM_readLong(ADDR_ANGLE6);
}

void hexapod_writeAnglesToEEPROM() {
  EEPROM_writeLong(ADDR_ANGLE1,robot.steps_to_zero[0]);
  EEPROM_writeLong(ADDR_ANGLE2,robot.steps_to_zero[1]);
  EEPROM_writeLong(ADDR_ANGLE3,robot.steps_to_zero[2]);
  EEPROM_writeLong(ADDR_ANGLE4,robot.steps_to_zero[3]);
  EEPROM_writeLong(ADDR_ANGLE5,robot.steps_to_zero[4]);
  EEPROM_writeLong(ADDR_ANGLE6,robot.steps_to_zero[5]);
}

/**
 * Build a virtual model of the hexapod shoulders for calculating angles later.
 */
void hexapod_build_shoulders() {
  Vector3 n,o,n1,o1;
  float c,s;
  int i;
  for(i=0;i<3;++i) {
    Arm &arma=robot.arms[i*2+0];
    Arm &armb=robot.arms[i*2+1];

    c=cos(i*TWOPI/3.0f);
    s=sin(i*TWOPI/3.0f);

    n=robot.ee.forward;
    o=robot.ee.up ^ robot.ee.forward;
    o.Normalize();

    n1 = n* c + o*s;
    o1 = n*-s + o*c;

    arma.shoulder = n1*B2S_X - o1*B2S_Y + robot.ee.up*B2S_Z;
    armb.shoulder = n1*B2S_X + o1*B2S_Y + robot.ee.up*B2S_Z;
    arma.elbow = n1*B2S_X - o1*(B2S_Y+BICEP_LENGTH) + robot.ee.up*B2S_Z;
    armb.elbow = n1*B2S_X + o1*(B2S_Y+BICEP_LENGTH) + robot.ee.up*B2S_Z;
    arma.shoulder_to_elbow=-o1;
    armb.shoulder_to_elbow=o1;
  }
  
#if VERBOSE > 0
  for(i=0;i<6;++i) {
    Arm &arm=robot.arms[i];
    Serial.print(i);
    Serial.print("\ts =");
    Serial.print(arm.shoulder.x);
    Serial.print(F(","));
    Serial.print(arm.shoulder.y);
    Serial.print(F(","));
    Serial.print(arm.shoulder.z);

    Serial.print("\te =");
    Serial.print(arm.elbow.x);
    Serial.print(F(","));
    Serial.print(arm.elbow.y);
    Serial.print(F(","));
    Serial.println(arm.elbow.z);
  }
#endif
}


/**
 * Update the end effector, the wrist positions, the elbows, and then calculate the new shoulder angles
 * @input mov final end effector position
 * @input rpy final end effector  roll pitch yaw (relative to base)
 */
void update_ik(Vector3 &mov,Vector3 &rpy) {
  hexapod_update_endeffector(mov,rpy);
  hexapod_update_wrists();
  hexapod_update_shoulder_angles();
}


/**
 * Update the end effector according to the desired motion
 * @input mov final end effector position
 * @input rpy final end effector roll pitch yaw (relative to base)
 */
void hexapod_update_endeffector(Vector3 &mov,Vector3 &rpy) {
  // translation
  robot.ee.pos = mov;
  
  // roll pitch & yaw  
  robot.ee.r=rpy.x*DEG2RAD;
  robot.ee.p=rpy.y*DEG2RAD;
  robot.ee.y=rpy.z*DEG2RAD;
  robot.ee.up.Set(0,0,1);
  robot.ee.forward.Set(1,0,0);
  robot.ee.left.Set(0,1,0);

  // roll
  Vector3 axis;
  axis.Set(1,0,0);
  robot.ee.up.Rotate(axis,robot.ee.r);
  robot.ee.forward.Rotate(axis,robot.ee.r);
  robot.ee.left.Rotate(axis,robot.ee.r);

  // pitch
  axis.Set(0,1,0);
  robot.ee.up.Rotate(axis,robot.ee.p);
  robot.ee.forward.Rotate(axis,robot.ee.p);
  robot.ee.left.Rotate(axis,robot.ee.p);

  // yaw
  axis.Set(0,0,1);
  robot.ee.up.Rotate(axis,robot.ee.y);
  robot.ee.forward.Rotate(axis,robot.ee.y);
  robot.ee.left.Rotate(axis,robot.ee.y);
}


/**
 * Update the wrist positions according to the end effector
 */
void hexapod_update_wrists() {
  Vector3 n1,o1;
  float c,s;
  int i;
  for(i=0;i<3;++i) {
    Arm &arma=robot.arms[i*2+0];
    Arm &armb=robot.arms[i*2+1];

    c=cos(i*TWOPI/3.0f);
    s=sin(i*TWOPI/3.0f);

    n1 = robot.ee.forward* c + robot.ee.left*s;
    o1 = robot.ee.forward*-s + robot.ee.left*c;

    arma.wrist = robot.ee.pos + robot.ee.relative + n1*T2W_X - o1*T2W_Y + robot.ee.up*T2W_Z;
    armb.wrist = robot.ee.pos + robot.ee.relative + n1*T2W_X + o1*T2W_Y + robot.ee.up*T2W_Z;
  }
  
#if VERBOSE > 0
  for(i=0;i<6;++i) {
    Arm &arm=robot.arms[i];
    Serial.print(i);
    Serial.print("\twrist =");
    Serial.print(arm.wrist.x);
    Serial.print(F(","));
    Serial.print(arm.wrist.y);
    Serial.print(F(","));
    Serial.println(arm.wrist.z);
  }
#endif
}


/**
 * update the elbow positions according to the wrists and shoulders, then find shoulder angle.
 */
void hexapod_update_shoulder_angles() {
  Vector3 ortho,w,wop,temp,r;
  float a,b,d,r1,r0,hh,y,x;
  
  int i;
  for(i=0;i<6;++i) {
    Arm &arm=robot.arms[i];
    
#if VERBOSE > 0
    Serial.print(i);
#endif

    // project wrist position onto plane of bicep (wop)
    ortho.x=cos((i/2)*TWOPI/3.0f);
    ortho.y=sin((i/2)*TWOPI/3.0f);
    ortho.z=0;
    
    w = arm.wrist - arm.shoulder;
    
    a=w | ortho;  //endeffector' distance
    wop = w - (ortho * a);
    //arm.wop.pos=wop + arm.shoulder;  // e' location
    //vector_add(arm.wop,wop,arm.shoulder);

    // because wop is projected onto the bicep plane, wop-elbow is not the same as wrist-elbow.
    // we need to find wop-elbow to calculate the angle at the shoulder.
    b=sqrt(FOREARM_LENGTH*FOREARM_LENGTH-a*a);  // e'j distance

    // use intersection of circles to find elbow point.
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    r1=b;  // circle 1 centers on e'
    r0=BICEP_LENGTH;  // circle 0 centers on shoulder
    d=wop.Length();
    // distance from shoulder to the midpoint between the two possible intersections
    a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );

#if VERBOSE > 0
    Serial.print("\tb =");
    Serial.println(b);
    Serial.print("\td =");
    Serial.println(d);
    Serial.print("\ta =");
    Serial.println(a);
#endif

    // normalize wop    
    wop /= d;
    // find the midpoint
    temp=arm.shoulder+(wop*a);
    // with a and r0 we can find h, the distance from midpoint to intersections.
    hh=sqrt(r0*r0-a*a);
    // get a normal to the line wop in the plane orthogonal to ortho
    r = ortho ^ wop;
    if(i%2==0) arm.elbow=temp + r * hh;
    else       arm.elbow=temp - r * hh;
    
    // use atan2 to find theta
    temp=arm.elbow-arm.shoulder;
    y=-temp.z;
    temp.z=0;
    x=temp.Length();
    if( ( arm.shoulder_to_elbow | temp ) < 0 ) x=-x;
    arm.angle= atan2(-y,x) * RAD2DEG;
#if VERBOSE > 0
    Serial.print(i);
    Serial.print("\tangle =");
    Serial.println(arm.angle);
#endif
  }
}


/**
 * Uses bresenham's line algorithm to move motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void robot_line(float newx,float newy,float newz,float newu,float newv,float neww,float new_feed_rate) {
  Vector3 endpos(newx,newy,newz);
  Vector3 endrpy(newu,newv,neww);

  endpos.x -= robot.tool_offset[robot.current_tool].x;
  endpos.y -= robot.tool_offset[robot.current_tool].y;
  endpos.z -= robot.tool_offset[robot.current_tool].z;
  
  Vector3 startpos=robot.ee.pos;
  Vector3 startrpy(robot.ee.r,robot.ee.p,robot.ee.y);
  Vector3 dpos=endpos-startpos;
  Vector3 drpy=endrpy-startrpy;
  Vector3 ipos, irpy;

  //@TODO: find which of the wrist positions moves the furthest, base everything on that.
  // ** BEGIN TOTAL JUNK
  int steps_pos = ceil( dpos.Length() * (float)MM_PER_SEGMENT );
  int steps_rpy = ceil( drpy.Length() * (float)MM_PER_SEGMENT );
  int pieces = ( steps_pos > steps_rpy ? steps_pos : steps_rpy);
  // ** END TOTAL JUNK

#if VERBOSE > 0
  outputvector(startpos,"start");
  outputvector(endpos,"end");
  Serial.print(pieces);
  Serial.println(F(" pieces."));
#endif

  int i;
  float f;
  for(i=1;i<pieces;++i) {
    f = (float)i / (float)pieces;
    ipos = startpos + dpos*f;
    irpy = startrpy + drpy*f;
    
    update_ik(ipos,irpy);
    motor_segment(new_feed_rate);
  }
  
  update_ik(endpos,endrpy);
  motor_segment(new_feed_rate);
  
  // remember the new position.
  robot.ee.pos = endpos;
  robot.ee.r = endrpy.x;
  robot.ee.p = endrpy.y;
  robot.ee.y = endrpy.z;  
}


void robot_position(float npx,float npy,float npz,float npu,float npv,float npw) {
  wait_for_segment_buffer_to_empty();
  
  robot.ee.pos.x=npx;
  robot.ee.pos.y=npy;
  robot.ee.pos.z=npz;
  robot.ee.r=npu;
  robot.ee.p=npv;
  robot.ee.y=npw;
  
  Vector3 rpy(robot.ee.r,robot.ee.p,robot.ee.y);
  // update kinematics (find angles)
  update_ik(robot.ee.pos,rpy);
  // Update the segment positions to match the new virtual position or they will go crazy
  // on the next robot_line().  Without this G92 is unpredictable.
  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    old_seg.a[i].step_count = 
    robot.arms[i].new_step = robot.arms[i].angle * MICROSTEP_PER_DEGREE;
  }

  // output new position
  motor_where();
}


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char hexapod_read_switches() {
  char i, hit=0;
  int state;
  
  for(i=0;i<6;++i) {
    state=digitalRead(robot.arms[i].limit_switch_pin);
#ifdef DEBUG_SWITCHES > 0
    Serial.print(state);
    Serial.print('\t');
#endif
    if(robot.arms[i].limit_switch_state != state) {
      robot.arms[i].limit_switch_state = state;
#ifdef DEBUG_SWITCHES > 0
      Serial.print(F("Switch "));
      Serial.println(i,DEC);
#endif
    }
    if(state == LOW) ++hit;
  }
#ifdef DEBUG_SWITCHES > 0
  Serial.print('\n');
#endif
  return hit;
}


/**
 * Move one motor in a given direction
 * @input the motor number [0...6]
 * @input the direction to move 1 for forward, -1 for backward
 **/
void hexapod_onestep(int motor,int dir) {
#if VERBOSE > 0
  Serial.print('A'+motor);
#endif
  Arm &a = robot.arms[motor];
  dir *= a.motor_scale;
  digitalWrite(a.motor_dir_pin,dir>0?LOW:HIGH);
  digitalWrite(a.motor_step_pin,HIGH);
  digitalWrite(a.motor_step_pin,LOW);
}


/**
 * Move the motors until they connect with the limit switches, then return to "zero" position.
 */
void robot_find_home() {
  Serial.println(F("Finding min..."));

  motor_enable();
  
  char i;
  // until all switches are hit
  while(hexapod_read_switches()<6) {
#if VERBOSE > 0
  Serial.println(hexapod_read_switches(),DEC);
#endif
    // for each stepper,
    for(i=0;i<6;++i) {
      // if this switch hasn't been hit yet
      if(robot.arms[i].limit_switch_state == HIGH) {
        // move "down"
        hexapod_onestep(i,-1);
      }
    }
    pause(250);
  }

  // The arms are 19.69 degrees from straight down when they hit the switcrobot.
  // @TODO: This could be better customized in firmware.
  hexapod_loadHomeAngles();
  
  Serial.println(F("Homing..."));
#if VERBOSE > 0
  Serial.print("steps=");
  Serial.println(steps_to_zero);
#endif
  char keepGoing;
  do {
    for(i=0;i<NUM_AXIES;++i) {
      keepGoing=0;
      if(robot.steps_to_zero[i]>0) {
        --robot.steps_to_zero[i];
        hexapod_onestep(i,1);
        keepGoing=1;
      }
    }
    pause(250);
  } while(keepGoing>0);
    
  // recalculate XYZ positions
  hexapod_setup();
  hexapod_loadHomeAngles();
}


//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
// This method assumes the start and end radius matcrobot.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void robot_arc(float cx,float cy,float x,float y,float z,float dir,float new_feed_rate) {
  Vector3 offset_pos = robot_get_end_plus_offset();
  
  // get radius
  float dx = offset_pos.x - cx;
  float dy = offset_pos.y - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = floor( len * MM_PER_SEGMENT );
 
  float nx, ny, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - offset_pos.z ) * scale + offset_pos.z;
    // send it to the planner
    robot_line(nx,ny,nz,robot.ee.r,robot.ee.p,robot.ee.y,new_feed_rate);
  }
  
  robot_line(x,y,z,robot.ee.r,robot.ee.p,robot.ee.y,new_feed_rate);
}



void robot_tool_offset(int axis,float x,float y,float z) {
  robot.tool_offset[axis].x=x;
  robot.tool_offset[axis].y=y;
  robot.tool_offset[axis].z=z;
}


Vector3 robot_get_end_plus_offset() {
  return Vector3(robot.tool_offset[robot.current_tool].x + robot.ee.pos.x,
                 robot.tool_offset[robot.current_tool].y + robot.ee.pos.y,
                 robot.tool_offset[robot.current_tool].z + robot.ee.pos.z);
}


void robot_tool_change(int tool_id) {
  if(tool_id < 0) tool_id=0;
  if(tool_id > NUM_TOOLS) tool_id=NUM_TOOLS;
  robot.current_tool=tool_id;
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void robot_where() {
  Vector3 offset = robot_get_end_plus_offset();
  output("X",offset.x);
  output("Y",offset.y);
  output("Z",offset.z);
  output("U",robot.ee.r);
  output("V",robot.ee.p);
  output("W",robot.ee.y);
  output("F",feed_rate);
  output("A",acceleration);
  Serial.println(mode_abs?"ABS":"REL");
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
