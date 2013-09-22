//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.
//#define DEBUG_SWITCHES       (1)

#define TWOPI        (PI*2.0)
#define DEG2RAD      (PI/180.0)
#define RAD2DEG      (180.0/PI)

#define VERSION              (1)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message we can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (150)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)
#define NUM_AXIES            (6)
#define MICROSTEPS           (16.0)

// measurements based on computer model of robot
#define BICEP_LENGTH         ( 5.000)
#define FOREARM_LENGTH       (16.750)
#define SWITCH_ANGLE         (19.690)
// top center to wrist hole: X7.635 Y+/-0.553 Z0.87
#define T2W_X                ( 7.635)
#define T2W_Y                ( 0.553)
#define T2W_Z                (-0.870)
// base center to shoulder hole: X8.093 Y+/-2.15 Z7.831
#define B2S_X                ( 8.093)
#define B2S_Y                ( 2.150)
#define B2S_Z                ( 6.618)

// distance moved by a single microstep
#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define CIRCUMFERENCE        (BICEP_LENGTH*PI*2.0)
#define MICROSTEP_DISTANCE   (CIRCUMFERENCE/MICROSTEPS_PER_TURN)
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "Vector3.h"


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
struct RobotEffector  {
  Vector3 up;
  Vector3 left;
  Vector3 forward;
  Vector3 pos;
  Vector3 relative;
  float r,p,y;  // roll, pitch, yaw
};


struct RobotArm {
  Vector3 shoulder;
  Vector3 elbow;
  Vector3 shoulder_to_elbow;
  Vector3 wrist;

  float angle;
  int last_angle;
  int angle_max;  // software limits
  int angle_min;  // software limits
  long last_step;
  long new_step;

  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int motor_scale;  // 1 or -1

  char limit_switch_pin;
  int limit_switch_state;
};


struct Hexapod {
  Vector3 base;
  RobotArm arms[6];
  RobotEffector ee;
 
  Vector3 v;  // max XYZ vel
  Vector3 destination;  // target XYZ position
};


// for motor_line()
struct Axis {
  long delta;
  long absdelta;
  int dir;
  long over;
  int motor;
};


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr=0;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?

#ifdef VERBOSE
char *letter="UVWXYZ";
#endif

Axis a[NUM_AXIES];  // for motor_line()
Axis atemp;  // for motor_line()

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
  
  for(i=0;i<6;++i) {  
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
  //h.ee.pos=h.ee.relative;

//#ifdef VERBOSE
  Serial.print("aa=");  Serial.println(aa);
  Serial.print("cc=");  Serial.println(cc);
  Serial.print("bb=");  Serial.println(bb);
  Serial.print("ee.z=");  Serial.println(h.ee.relative.z);
//#endif

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
    RobotArm &arma=h.arms[i*2+0];
    RobotArm &armb=h.arms[i*2+1];

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
    RobotArm &arm=h.arms[i];
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
    RobotArm &arma=h.arms[i*2+0];
    RobotArm &armb=h.arms[i*2+1];

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
    RobotArm &arm=h.arms[i];
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
    RobotArm &arm=h.arms[i];
    
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
void hexapod_line(float newx,float newy,float newz,float newu,float newv,float neww) {
  motor_enable();
  
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
  long steps_pos=ceil(dpos.Length()/MICROSTEP_DISTANCE);
  long steps_rpy=ceil(drpy.Length()/MICROSTEP_DISTANCE);
  long steps = ( steps_pos > steps_rpy ? steps_pos : steps_rpy) / 100.0;
  // ** END TOTAL JUNK

//#ifdef VERBOSE  
  Serial.print(steps);
  Serial.println(" steps.");
//#endif

  if(steps==0) steps=1;

  long i;
  int j;
  
  for(i=0;i<=steps;++i) {
    ipos=startpos+dpos*((float)i/steps);
    irpy=startrpy+drpy*((float)i/steps);
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

    motor_line(h.arms[0].new_step,
               h.arms[1].new_step,
               h.arms[2].new_step,
               h.arms[3].new_step,
               h.arms[4].new_step,
               h.arms[5].new_step);
  }

  // This does not take into account movements of a fraction of a step.  They will be misreported and lead to error.
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
  // @TODO: Until motor positions can match hexapod position G92 *will* have unintended effects.
}

  
/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for ms values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MAX_FEEDRATE;
  }
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MIN_FEEDRATE;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
}


/**
 * Set the motor position in number of steps
 */
void motor_position(long n0,long n1,long n2,long n3,long n4,long n5) {
  // here is a good place to add sanity tests
  h.arms[0].last_step=n0;
  h.arms[1].last_step=n1;
  h.arms[2].last_step=n2;
  h.arms[3].last_step=n3;
  h.arms[4].last_step=n4;
  h.arms[5].last_step=n5;
}


/**
 * Move one motor in a given direction
 * @input the motor number [0...6]
 * @input the direction to move 1 for forward, -1 for backward
 **/
void onestep(int motor,int dir) {
#ifdef VERBOSE
  Serial.print(letter[motor]);
#endif
  dir *= h.arms[motor].motor_scale;
  digitalWrite(h.arms[motor].motor_dir_pin,dir>0?LOW:HIGH);
  digitalWrite(h.arms[motor].motor_step_pin,LOW);
  digitalWrite(h.arms[motor].motor_step_pin,HIGH);
}


/**
 * Grips the power on the motors
 **/
void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(h.arms[i].motor_enable_pin,LOW);
  }
}


/**
 * Releases the power on the motors
 **/
void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(h.arms[i].motor_enable_pin,HIGH);
  }
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void motor_line(long n0,long n1,long n2,long n3,long n4,long n5) {
  a[0].delta = n0-h.arms[0].last_step;
  a[1].delta = n1-h.arms[1].last_step;
  a[2].delta = n2-h.arms[2].last_step;
  a[3].delta = n3-h.arms[3].last_step;
  a[4].delta = n4-h.arms[4].last_step;
  a[5].delta = n5-h.arms[5].last_step;
  
  long i,j;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].motor = i;
    a[i].dir = (a[i].delta > 0 ? 1:-1);
    a[i].absdelta = abs(a[i].delta);
  }
  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  // sort the axies with the fastest mover at the front of the list
  int jmax;
  for(i=0;i<NUM_AXIES;++i) {
    jmax=i;
    for(j=i+1;j<NUM_AXIES;++j) {
      if(a[j].absdelta>a[i].absdelta) {
        jmax=j;
      }
    }
    if(i!=jmax) {
      memcpy(&atemp  ,&a[i]   ,sizeof(Axis));
      memcpy(&a[i]   ,&a[jmax],sizeof(Axis));
      memcpy(&a[jmax],&atemp  ,sizeof(Axis));
    }
    a[i].over=0;
  }
  
  for(i=0;i<a[0].absdelta;++i) {
    onestep(a[0].motor,a[0].dir);
    
    for(j=1;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= a[0].absdelta) {
        a[j].over -= a[0].absdelta;
        onestep(a[j].motor,a[j].dir);
      }
    }
    pause(step_delay);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  motor_position(n0,n1,n2,n3,n4,n5);
}


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
 * print the current position, feedrate, and absolute mode.
 */
void hexapod_where() {
  output("X",h.ee.pos.x);
  output("Y",h.ee.pos.y);
  output("Z",h.ee.pos.z);
  output("U",h.ee.r);
  output("V",h.ee.p);
  output("W",h.ee.y);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * print the current motor positions in steps
 */
void motor_where() {
  output("0",h.arms[0].last_step);
  output("1",h.arms[1].last_step);
  output("2",h.arms[2].last_step);
  output("3",h.arms[3].last_step);
  output("4",h.arms[4].last_step);
  output("5",h.arms[5].last_step);
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("StewartPlatform v4-2"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("M17/M18; - enable/disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("F, G00, G01, G04, G28, G90, G91 as described by http://en.wikipedia.org/wiki/G-code"));
  // See hexapod_position() for note about why G92 is removed
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
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
  case 28:  find_home();  break;  
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
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char read_switches() {
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
 * Move the motors until they connect with the limit switches, then return to "zero" position.
 */
void find_home() {
  Serial.println(F("Finding min..."));

  motor_enable();
  
  char i;
  // until all switches are hit
  while(read_switches()<6) {
#ifdef VERBOSE
  Serial.println(read_switches(),DEC);
#endif
    // for each stepper,
    for(i=0;i<6;++i) {
      // if this switch hasn't been hit yet
      if(h.arms[i].limit_switch_state == HIGH) {
        // move "down"
        onestep(i,-1);
      }
    }
    pause(step_delay);
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
      onestep(i,1);
    }
    pause(step_delay);
  }
  hexapod_position(0,0,0,0,0,0);
  motor_position(0,0,0,0,0,0);
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  help();  // say hello

  hexapod_setup();
  motor_enable();
  feedrate(800);  // set default speed
  hexapod_position(0,0,0,0,0,0);
  motor_position(0,0,0,0,0,0);

  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
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
    processCommand();  // do something with the command
    ready();
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
