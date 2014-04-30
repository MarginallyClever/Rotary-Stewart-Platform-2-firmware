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
Segment *working_seg = NULL;
volatile int current_segment=0;
volatile int last_segment=0;
int step_multiplier;

// used by timer1 to optimize interrupt inner loop
int delta[NUM_AXIES];
int over[NUM_AXIES];
int steps_total;
int steps_taken;
int accel_until,decel_after;


long prescalers[] = {CLOCK_FREQ /   1,
                     CLOCK_FREQ /   8,
                     CLOCK_FREQ /  64,
                     CLOCK_FREQ / 256,
                     CLOCK_FREQ /1024};

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


FORCE_INLINE int get_next_segment(int i) {
  return ( i + 1 ) & ( MAX_SEGMENTS - 1 );
}


FORCE_INLINE int get_prev_segment(int i) {
  return ( i + MAX_SEGMENTS - 1 ) & ( MAX_SEGMENTS - 1 );
}


/**
 * Add a segment to the line buffer if there is room.
 */
void motor_prepare_segment(int n0,int n1,int n2,int n3,int n4,int n5,float new_feed_rate) {
  int next_segment = get_next_segment(last_segment);
  while( next_segment == current_segment ) {
    // the buffer is full, we are way ahead of the motion system
    delay(1);
  }

  Segment &new_seg = line_segments[last_segment];
  new_seg.a[0].step_count = n0;
  new_seg.a[1].step_count = n1;
  new_seg.a[2].step_count = n2;
  new_seg.a[3].step_count = n3;
  new_seg.a[4].step_count = n4;
  new_seg.a[5].step_count = n5;
  
  int i;

  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  new_seg.a[0].delta = n0 - old_seg.a[0].step_count;
  new_seg.a[1].delta = n1 - old_seg.a[1].step_count;
  new_seg.a[2].delta = n2 - old_seg.a[2].step_count;
  new_seg.a[3].delta = n3 - old_seg.a[3].step_count;
  new_seg.a[4].delta = n4 - old_seg.a[4].step_count;
  new_seg.a[5].delta = n5 - old_seg.a[5].step_count;

  new_seg.steps_total=0;
  new_seg.feed_rate_start=
  new_seg.feed_rate_end  = new_feed_rate;

  for(i=0;i<NUM_AXIES;++i) {
    new_seg.a[i].dir = (new_seg.a[i].delta * h.arms[i].motor_scale ) > 0 ? LOW:HIGH;
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    if( new_seg.steps_total < new_seg.a[i].absdelta ) {
      new_seg.steps_total = new_seg.a[i].absdelta;
    }
  }

  if( new_seg.steps_total==0 ) return;

  new_seg.steps_taken = 0;
  new_seg.decel_after = 
  new_seg.accel_until = new_seg.steps_total / 2;
  
#ifdef VERBOSE
  Serial.print(F("At "));  Serial.println(current_segment);
  Serial.print(F("Adding "));  Serial.println(last_segment);
  Serial.print(F("Steps= "));  Serial.println(new_seg.steps_left);
#endif

  if( current_segment==last_segment ) {
    timer_set_frequency(new_feed_rate);
  }
  
  last_segment = next_segment;
}


/**
 * Set the clock 1 timer frequency.
 * @input desired_freq_hz the desired frequency
 */
void timer_set_frequency(long desired_freq_hz) {
  // Source: https://github.com/MarginallyClever/ArduinoTimerInterrupt
  // Different clock sources can be selected for each timer independently. 
  // To calculate the timer frequency (for example 2Hz using timer1) you will need:
  
  if(desired_freq_hz > 20000 ) {
    step_multiplier = 4;
    desired_freq_hz >>= 2;
  } else if(desired_freq_hz > 20000 ) {
    step_multiplier = 2;
    desired_freq_hz >>= 1;
  } else {
    step_multiplier=1;
  }
  
  //  CPU frequency 16Mhz for Arduino
  //  maximum timer counter value (256 for 8bit, 65536 for 16bit timer)
  int prescaler_index=-1;
  long counter_value;
  do {
    ++prescaler_index;
    //  Divide CPU frequency through the choosen prescaler (16000000 / 256 = 62500)
    //  Divide result through the desired frequency (62500 / 2Hz = 31250)
    counter_value = prescalers[prescaler_index] / desired_freq_hz;
    //  Verify counter_value < maximum timer. if fail, choose bigger prescaler.
  } while(counter_value > MAX_COUNTER && prescaler_index<4);
  
  if( prescaler_index>=5 ) {
    Serial.println(F("Timer could not be set: Desired frequency out of bounds."));
    return;
  }
  
  prescaler_index++;

  // disable global interrupts
  noInterrupts();
  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set entire TCCR1B register to 0
  TCCR1B = 0;
  // set the overflow clock to 0
  TCNT1  = 0;
  // set compare match register to desired timer count
  OCR1A = counter_value;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for prescaler
  TCCR1B |= ( (( prescaler_index&0x1 )   ) << CS10);
  TCCR1B |= ( (( prescaler_index&0x2 )>>1) << CS11);
  TCCR1B |= ( (( prescaler_index&0x4 )>>2) << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts
  interrupts();
}

 
/**
 * Process all line segments in the ring buffer.  Uses bresenham's line algorithm to move all motors.
 */
ISR(TIMER1_COMPA_vect) {
  // segment buffer empty? do nothing
  if( working_seg == NULL ) {
    working_seg = segment_get_working();
    if( working_seg != NULL ) {
      // New segment!
      // set the direction pins
      digitalWrite( MOTOR_0_DIR_PIN, working_seg->a[0].dir );
      digitalWrite( MOTOR_1_DIR_PIN, working_seg->a[1].dir );
      digitalWrite( MOTOR_2_DIR_PIN, working_seg->a[2].dir );
      digitalWrite( MOTOR_3_DIR_PIN, working_seg->a[3].dir );
      digitalWrite( MOTOR_4_DIR_PIN, working_seg->a[4].dir );
      digitalWrite( MOTOR_5_DIR_PIN, working_seg->a[5].dir );
      
      // set frequency to segment feed rate
      timer_set_frequency(working_seg->feed_rate_start);
      
      // defererencing some data so the loop runs faster.
      steps_total=working_seg->steps_total;
      steps_taken=0;
      delta[0] = working_seg->a[0].absdelta;
      delta[1] = working_seg->a[1].absdelta;
      delta[2] = working_seg->a[2].absdelta;
      delta[3] = working_seg->a[3].absdelta;
      delta[4] = working_seg->a[4].absdelta;
      delta[5] = working_seg->a[5].absdelta;
      memset(over,0,sizeof(int)*NUM_AXIES);
      accel_until=working_seg->accel_until;
      decel_after=working_seg->decel_after;
    }
  }
  
  if( working_seg != NULL ) {
    // move each axis
    for(int i=0;i<step_multiplier;++i) {
      // M0
      over[0] += delta[0];
      if(over[0] >= steps_total) {
        digitalWrite(MOTOR_0_STEP_PIN,LOW);
        over[0] -= steps_total;
        digitalWrite(MOTOR_0_STEP_PIN,HIGH);
      }
      // M1
      over[1] += delta[1];
      if(over[1] >= steps_total) {
        digitalWrite(MOTOR_1_STEP_PIN,LOW);
        over[1] -= steps_total;
        digitalWrite(MOTOR_1_STEP_PIN,HIGH);
      }
      // M2
      over[2] += delta[2];
      if(over[2] >= steps_total) {
        digitalWrite(MOTOR_2_STEP_PIN,LOW);
        over[2] -= steps_total;
        digitalWrite(MOTOR_2_STEP_PIN,HIGH);
      }
      // M3
      over[3] += delta[3];
      if(over[3] >= steps_total) {
        digitalWrite(MOTOR_3_STEP_PIN,LOW);
        over[3] -= steps_total;
        digitalWrite(MOTOR_3_STEP_PIN,HIGH);
      }
      // M4
      over[4] += delta[4];
      if(over[4] >= steps_total) {
        digitalWrite(MOTOR_4_STEP_PIN,LOW);
        over[4] -= steps_total;
        digitalWrite(MOTOR_4_STEP_PIN,HIGH);
      }
      // M5
      over[5] += delta[5];
      if(over[5] >= steps_total) {
        digitalWrite(MOTOR_5_STEP_PIN,LOW);
        over[5] -= steps_total;
        digitalWrite(MOTOR_5_STEP_PIN,HIGH);
      }
    }
    
    // make a step
    steps_taken++;

    // accel
    if( steps_taken <= accel_until ) {
//      OCR1A-=200;
    } else if( steps_taken > decel_after ) {
//      OCR1A+=200;
    }

    // Is this segment done?
    if( steps_taken >= steps_total ) {
      // Move on to next segment without wasting an interrupt tick.
      working_seg = NULL;
      current_segment = get_next_segment(current_segment);
    }
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
