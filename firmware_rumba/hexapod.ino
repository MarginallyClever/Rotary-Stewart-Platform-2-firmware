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
Hexapod h;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * set up the digital pins and call hexapod_build_model()
 */
void hexapod_setup() {
  int i;
  
  // set up the pins
  h.arms[0].motor_step_pin=17;
  h.arms[0].motor_dir_pin=16;
  h.arms[0].motor_enable_pin=48;
  h.arms[0].limit_switch_pin=37;

  h.arms[1].motor_step_pin=54;
  h.arms[1].motor_dir_pin=47;
  h.arms[1].motor_enable_pin=55;
  h.arms[1].limit_switch_pin=36;

  h.arms[2].motor_step_pin=57;
  h.arms[2].motor_dir_pin=56;
  h.arms[2].motor_enable_pin=62;
  h.arms[2].limit_switch_pin=35;

  h.arms[3].motor_step_pin=23;
  h.arms[3].motor_dir_pin=22;
  h.arms[3].motor_enable_pin=27;
  h.arms[3].limit_switch_pin=34;

  h.arms[4].motor_step_pin=26;
  h.arms[4].motor_dir_pin=25;
  h.arms[4].motor_enable_pin=24;
  h.arms[4].limit_switch_pin=33;

  h.arms[5].motor_step_pin=29;
  h.arms[5].motor_dir_pin=28;
  h.arms[5].motor_enable_pin=39;
  h.arms[5].limit_switch_pin=32;
  
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    h.arms[i].motor_scale=((i%2)? -1:1);
    pinMode(h.arms[i].motor_step_pin,OUTPUT);
    pinMode(h.arms[i].motor_dir_pin,OUTPUT);
    pinMode(h.arms[i].motor_enable_pin,OUTPUT);
    // set the switch pin
    h.arms[i].limit_switch_state=HIGH;
    pinMode(h.arms[i].limit_switch_pin,INPUT);
    digitalWrite(h.arms[i].limit_switch_pin,HIGH);
  }
  
  hexapod_build_model();
}


/**
 * Build a virtual model of the hexapod at home position.
 */
void hexapod_build_model() {
  h.ee.relative.Set(0,0,0);
  Vector3 zero(0,0,0);
  hexapod_update_endeffector(zero,zero);
  hexapod_build_shoulders();
  hexapod_update_wrists();

  // find the starting height of the end effector at home position
  // @TODO: project wrist-on-bicep to get more accurate distance
  Vector3 el=h.arms[0].elbow;
  Vector3 wr=h.arms[0].wrist;
  float aa=(el.y-wr.y);
  float cc=FOREARM_LENGTH;
  float bb=sqrt((cc*cc)-(aa*aa));
  aa=el.x-wr.x;
  cc=bb;
  bb=sqrt((cc*cc)-(aa*aa));
  h.ee.relative.z=bb+B2S_Z-T2W_Z;

  hexapod_update_ik(h.ee.relative,zero);
}


/**
 * Build a virtual model of the hexapod shoulders for calculating angles later.
 */
void hexapod_build_shoulders() {
  Vector3 n,o,n1,o1;
  float c,s;
  int i;
  for(i=0;i<3;++i) {
    Arm &arma=h.arms[i*2+0];
    Arm &armb=h.arms[i*2+1];

    c=cos(i*TWOPI/3.0f);
    s=sin(i*TWOPI/3.0f);

    n=h.ee.forward;
    o=h.ee.up ^ h.ee.forward;
    o.Normalize();

    n1 = n* c + o*s;
    o1 = n*-s + o*c;

    arma.shoulder = n1*B2S_X - o1*B2S_Y + h.ee.up*B2S_Z;
    armb.shoulder = n1*B2S_X + o1*B2S_Y + h.ee.up*B2S_Z;
    arma.elbow = n1*B2S_X - o1*(B2S_Y+BICEP_LENGTH) + h.ee.up*B2S_Z;
    armb.elbow = n1*B2S_X + o1*(B2S_Y+BICEP_LENGTH) + h.ee.up*B2S_Z;
    arma.shoulder_to_elbow=-o1;
    armb.shoulder_to_elbow=o1;
  }
  
#ifdef VERBOSE
  for(i=0;i<6;++i) {
    Arm &arm=h.arms[i];
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
void hexapod_update_ik(Vector3 &mov,Vector3 &rpy) {
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
  h.ee.pos = mov;
  
  // roll pitch & yaw  
  h.ee.r=rpy.x*DEG2RAD;
  h.ee.p=rpy.y*DEG2RAD;
  h.ee.y=rpy.z*DEG2RAD;
  h.ee.up.Set(0,0,1);
  h.ee.forward.Set(1,0,0);
  h.ee.left.Set(0,1,0);

  // roll
  Vector3 axis;
  axis.Set(1,0,0);
  h.ee.up.Rotate(axis,h.ee.r);
  h.ee.forward.Rotate(axis,h.ee.r);
  h.ee.left.Rotate(axis,h.ee.r);

  // pitch
  axis.Set(0,1,0);
  h.ee.up.Rotate(axis,h.ee.p);
  h.ee.forward.Rotate(axis,h.ee.p);
  h.ee.left.Rotate(axis,h.ee.p);

  // yaw
  axis.Set(0,0,1);
  h.ee.up.Rotate(axis,h.ee.y);
  h.ee.forward.Rotate(axis,h.ee.y);
  h.ee.left.Rotate(axis,h.ee.y);
}


/**
 * Update the wrist positions according to the end effector
 */
void hexapod_update_wrists() {
  Vector3 n,o,n1,o1;
  float c,s;
  int i;
  for(i=0;i<3;++i) {
    Arm &arma=h.arms[i*2+0];
    Arm &armb=h.arms[i*2+1];

    c=cos(i*TWOPI/3.0f);
    s=sin(i*TWOPI/3.0f);

    n=h.ee.forward;
    o=h.ee.up ^ h.ee.forward;
    o.Normalize();

    n1 = n* c + o*s;
    o1 = n*-s + o*c;

    arma.wrist = h.ee.pos + n1*T2W_X - o1*T2W_Y + h.ee.up*T2W_Z;
    armb.wrist = h.ee.pos + n1*T2W_X + o1*T2W_Y + h.ee.up*T2W_Z;
  }
  
#ifdef VERBOSE
  for(i=0;i<6;++i) {
    Arm &arm=h.arms[i];
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
    Arm &arm=h.arms[i];
    
#ifdef VERBOSE
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

#ifdef VERBOSE
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
#ifdef VERBOSE
    Serial.print(i);
    Serial.print("\tangle =");
    Serial.println(arm.angle);
#endif
  }
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void hexapod_line(float newx,float newy,float newz,float newu,float newv,float neww,float new_feed_rate) {
  Vector3 endpos(newx,newy,newz);
  Vector3 endrpy(newu,newv,neww);

  endpos+=h.ee.relative;
  
  Vector3 startpos=h.ee.pos+h.ee.relative;
  Vector3 startrpy(h.ee.r,h.ee.p,h.ee.y);
  Vector3 dpos=endpos-startpos;
  Vector3 drpy=endrpy-startrpy;
  Vector3 ipos, irpy;

  //@TODO: find which of the wrist positions moves the furthest, base everything on that.
  // ** BEGIN TOTAL JUNK
  long steps_pos=ceil(dpos.Length()/SEGMENTS_PER_CM);
  long steps_rpy=ceil(drpy.Length()/SEGMENTS_PER_DEG);
  long steps = ( steps_pos > steps_rpy ? steps_pos : steps_rpy);
  if(steps>MAX_SEGMENTS) steps=MAX_SEGMENTS;
  // ** END TOTAL JUNK

  if( steps>=MAX_SEGMENTS-1) steps=MAX_SEGMENTS-2;

#ifdef VERBOSE  
  Serial.print(steps);
  Serial.println(F(" steps."));
#endif

  if(steps==0) return;
  
  float istep = 1.0/(float)steps;

  long i;
  int j;
  
  for(i=0;i<=steps;++i) {
    ipos=startpos+dpos*(i*istep);
    irpy=startrpy+drpy*(i*istep);
    
    hexapod_update_ik(ipos,irpy);

#ifdef VERBOSE    
    Serial.print(i);
    Serial.print(" = ");    Serial.print(h.arms[0].angle);
    Serial.print(F(", "));  Serial.print(h.arms[1].angle);
    Serial.print(F(", "));  Serial.print(h.arms[2].angle);
    Serial.print(F(", "));  Serial.print(h.arms[3].angle);
    Serial.print(F(", "));  Serial.print(h.arms[4].angle);
    Serial.print(F(", "));  Serial.println(h.arms[5].angle);
#endif

    // convert angle to motor steps
    for(j=0;j<6;++j) {
      h.arms[j].new_step = h.arms[j].angle * MICROSTEP_PER_DEGREE;
    }
    
#ifdef VERBOSE    
    Serial.print(i);
    Serial.print(" = ");    Serial.print(h.arms[0].new_step);
    Serial.print(F(", "));  Serial.print(h.arms[1].new_step);
    Serial.print(F(", "));  Serial.print(h.arms[2].new_step);
    Serial.print(F(", "));  Serial.print(h.arms[3].new_step);
    Serial.print(F(", "));  Serial.print(h.arms[4].new_step);
    Serial.print(F(", "));  Serial.println(h.arms[5].new_step);
#endif

    motor_prepare_segment(h.arms[0].new_step,
                          h.arms[1].new_step,
                          h.arms[2].new_step,
                          h.arms[3].new_step,
                          h.arms[4].new_step,
                          h.arms[5].new_step,new_feed_rate);
  }
  
  // @TODO: This does not take into account movements of a fraction of a step.  They will be misreported and lead to error.
  hexapod_position(newx,newy,newz,newu,newv,neww);
}


void hexapod_position(float npx,float npy,float npz,float npu,float npv,float npw) {
  h.ee.pos.x=npx;
  h.ee.pos.y=npy;
  h.ee.pos.z=npz;
  h.ee.r=npu;
  h.ee.p=npv;
  h.ee.y=npw;
  // @TODO: Update the motor positions to match the new virtual position or they will go crazy on the next hexapod_line()
  // @TODO: Until motor positions can match hexapod position G92 will have unintended effects.
}


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char hexapod_read_switches() {
  char i, hit=0;
  int state;
  
  for(i=0;i<6;++i) {
    state=digitalRead(h.arms[i].limit_switch_pin);
#ifdef DEBUG_SWITCHES
    Serial.print(state);
    Serial.print('\t');
#endif
    if(h.arms[i].limit_switch_state != state) {
      h.arms[i].limit_switch_state = state;
#ifdef DEBUG_SWITCHES
      Serial.print(F("Switch "));
      Serial.println(i,DEC);
#endif
    }
    if(state == LOW) ++hit;
  }
#ifdef DEBUG_SWITCHES
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
#ifdef VERBOSE
  Serial.print(letter[motor]);
#endif
  Arm &a = h.arms[motor];
  dir *= a.motor_scale;
  digitalWrite(a.motor_dir_pin,dir>0?LOW:HIGH);
  digitalWrite(a.motor_step_pin,HIGH);
  digitalWrite(a.motor_step_pin,LOW);
}


/**
 * Move the motors until they connect with the limit switches, then return to "zero" position.
 */
void hexapod_find_home() {
  Serial.println(F("Finding min..."));

  //motor_enable();
  
  char i;
  // until all switches are hit
  while(hexapod_read_switches()<6) {
#ifdef VERBOSE
  Serial.println(hexapod_read_switches(),DEC);
#endif
    // for each stepper,
    for(i=0;i<6;++i) {
      // if this switch hasn't been hit yet
      if(h.arms[i].limit_switch_state == HIGH) {
        // move "down"
        hexapod_onestep(i,-1);
      }
    }
    pause(150);
  }

  // The arms are 19.69 degrees from straight down when they hit the switch.
  // @TODO: This could be better customized in firmware.
  float horizontal = 90.00 - SWITCH_ANGLE;
  long steps_to_zero = MICROSTEP_PER_DEGREE * horizontal;
  Serial.println(F("Homing..."));
#ifdef VERBOSE
  Serial.print("steps=");
  Serial.println(step_size);
#endif

  for(;steps_to_zero>0;--steps_to_zero) {
    for(i=0;i<6;++i) {
      hexapod_onestep(i,1);
    }
    pause(150);
  }
  hexapod_position(0,0,0,0,0,0);
  motor_position(0,0,0,0,0,0);
}


//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void hexapod_arc(float cx,float cy,float x,float y,float z,float dir,float new_feed_rate) {
  Vector3 offset_pos = hexapod_get_end_plus_offset();
  
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

  int i, segments = floor( len / SEGMENTS_PER_DEG );
 
  float nx, ny, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - offset_pos.z ) * scale + offset_pos.z;
    // send it to the planner
    hexapod_line(nx,ny,nz,h.ee.r,h.ee.p,h.ee.y,new_feed_rate);
  }
  
  hexapod_line(x,y,z,h.ee.r,h.ee.p,h.ee.y,new_feed_rate);
}



void hexapod_tool_offset(int axis,float x,float y,float z) {
  h.tool_offset[axis].x=x;
  h.tool_offset[axis].y=y;
  h.tool_offset[axis].z=z;
}


Vector3 hexapod_get_end_plus_offset() {
  return Vector3(h.tool_offset[h.current_tool].x + h.ee.pos.x,
                 h.tool_offset[h.current_tool].y + h.ee.pos.y,
                 h.tool_offset[h.current_tool].z + h.ee.pos.z);
}


void hexapod_tool_change(int tool_id) {
  if(tool_id < 0) tool_id=0;
  if(tool_id > NUM_TOOLS) tool_id=NUM_TOOLS;
  h.current_tool=tool_id;
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
