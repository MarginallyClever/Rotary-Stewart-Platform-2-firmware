#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.
//#define DEBUG_SWITCHES       (1)


#define VERSION              (1)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message we can store?

#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (150)  // depends on electronics

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

#define STEPS_PER_CM         (10)
#define STEPS_PER_DEG        (5)


// ** Nothing below this line needs to be configured **


// convenience macros
#define NUM_AXIES            (6)

#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define CIRCUMFERENCE        (BICEP_LENGTH*TWOPI)
#define MICROSTEP_DISTANCE   (CIRCUMFERENCE/MICROSTEPS_PER_TURN)  // distance elbow moves in a single microstep
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)

#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)

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
