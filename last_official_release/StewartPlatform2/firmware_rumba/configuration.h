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
#define VERBOSE              (0)  // increase this number to get more output
//#define DEBUG_SWITCHES       (1)

#define EEPROM_VERSION       (1)  // firmware version

#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message we can store?

#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MICROSTEPS           (16.0)

#define MAX_FEEDRATE         (40000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (500)
#define DEFAULT_FEEDRATE     (3000.0)
#define DEFAULT_ACCELERATION (5)

#define NUM_AXIES            (6)

// related to number of instructions that can be buffered.  must be a power of two > 1.
#define MAX_SEGMENTS         (64)  // must be a power of 2
// for splitting lines and look-ahead
#define MM_PER_SEGMENT       (3)

// convenience macros
#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)


// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK           (1000)
// timing
#define CLOCK_FREQ           (16000000L)
#define MAX_COUNTER          (65536L)


// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START



// hardware/software pins
#define MOTOR_0_DIR_PIN    (16)
#define MOTOR_0_STEP_PIN   (17)
#define MOTOR_0_ENABLE_PIN (48)
#define MOTOR_0_LIMIT_PIN  (37)

#define MOTOR_1_DIR_PIN    (47)
#define MOTOR_1_STEP_PIN   (54)
#define MOTOR_1_ENABLE_PIN (55)
#define MOTOR_1_LIMIT_PIN  (36)

#define MOTOR_2_DIR_PIN    (56)
#define MOTOR_2_STEP_PIN   (57)
#define MOTOR_2_ENABLE_PIN (62)
#define MOTOR_2_LIMIT_PIN  (35)

#define MOTOR_3_DIR_PIN    (22)
#define MOTOR_3_STEP_PIN   (23)
#define MOTOR_3_ENABLE_PIN (27)
#define MOTOR_3_LIMIT_PIN  (34)

#define MOTOR_4_DIR_PIN    (25)
#define MOTOR_4_STEP_PIN   (26)
#define MOTOR_4_ENABLE_PIN (24)
#define MOTOR_4_LIMIT_PIN  (33)

#define MOTOR_5_DIR_PIN    (28)
#define MOTOR_5_STEP_PIN   (29)
#define MOTOR_5_ENABLE_PIN (39)
#define MOTOR_5_LIMIT_PIN  (32)


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
