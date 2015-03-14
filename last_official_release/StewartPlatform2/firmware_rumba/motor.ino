//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#if VERBOSE > 0
char *letter="UVWXYZ";
#endif


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void motor_setup() {
  int i;
  
  // set up the pins
  robot.arms[0].motor_step_pin=MOTOR_0_STEP_PIN;
  robot.arms[0].motor_dir_pin=MOTOR_0_DIR_PIN;
  robot.arms[0].motor_enable_pin=MOTOR_0_ENABLE_PIN;
  robot.arms[0].limit_switch_pin=MOTOR_0_LIMIT_PIN;

  robot.arms[1].motor_step_pin=MOTOR_1_STEP_PIN;
  robot.arms[1].motor_dir_pin=MOTOR_1_DIR_PIN;
  robot.arms[1].motor_enable_pin=MOTOR_1_ENABLE_PIN;
  robot.arms[1].limit_switch_pin=MOTOR_1_LIMIT_PIN;
  
  robot.arms[2].motor_step_pin=MOTOR_2_STEP_PIN;
  robot.arms[2].motor_dir_pin=MOTOR_2_DIR_PIN;
  robot.arms[2].motor_enable_pin=MOTOR_2_ENABLE_PIN;
  robot.arms[2].limit_switch_pin=MOTOR_2_LIMIT_PIN;
  
  robot.arms[3].motor_step_pin=MOTOR_3_STEP_PIN;
  robot.arms[3].motor_dir_pin=MOTOR_3_DIR_PIN;
  robot.arms[3].motor_enable_pin=MOTOR_3_ENABLE_PIN;
  robot.arms[3].limit_switch_pin=MOTOR_3_LIMIT_PIN;
  
  robot.arms[4].motor_step_pin=MOTOR_4_STEP_PIN;
  robot.arms[4].motor_dir_pin=MOTOR_4_DIR_PIN;
  robot.arms[4].motor_enable_pin=MOTOR_4_ENABLE_PIN;
  robot.arms[4].limit_switch_pin=MOTOR_4_LIMIT_PIN;
  
  robot.arms[5].motor_step_pin=MOTOR_5_STEP_PIN;
  robot.arms[5].motor_dir_pin=MOTOR_5_DIR_PIN;
  robot.arms[5].motor_enable_pin=MOTOR_5_ENABLE_PIN;
  robot.arms[5].limit_switch_pin=MOTOR_5_LIMIT_PIN;
  
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    robot.arms[i].motor_scale=((i%2)? -1:1);
    pinMode(robot.arms[i].motor_step_pin,OUTPUT);
    pinMode(robot.arms[i].motor_dir_pin,OUTPUT);
    pinMode(robot.arms[i].motor_enable_pin,OUTPUT);
    // set the switch pin
    robot.arms[i].limit_switch_state=HIGH;
    pinMode(robot.arms[i].limit_switch_pin,INPUT);
    digitalWrite(robot.arms[i].limit_switch_pin,HIGH);
  }
}


void motor_segment(float new_feed_rate) {
  int i;
  // convert angle to motor steps
  for(i=0;i<NUM_AXIES;++i) {
    robot.arms[i].new_step = robot.arms[i].angle * MICROSTEP_PER_DEGREE;
  }

#if VERBOSE > 0
  Serial.print(robot.arms[0].new_step);  Serial.print(" ");
  Serial.print(robot.arms[1].new_step);  Serial.print(" ");
  Serial.print(robot.arms[2].new_step);  Serial.print(" ");
  Serial.print(robot.arms[3].new_step);  Serial.print(" ");
  Serial.print(robot.arms[4].new_step);  Serial.print(" ");
  Serial.print(robot.arms[5].new_step);  Serial.print("\n");
#endif

  motor_prepare_segment(robot.arms[0].new_step,
                        robot.arms[1].new_step,
                        robot.arms[2].new_step,
                        robot.arms[3].new_step,
                        robot.arms[4].new_step,
                        robot.arms[5].new_step,new_feed_rate);
}


/**
 * Grips the power on the motors
 **/
void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(robot.arms[i].motor_enable_pin,LOW);
  }
}


/**
 * Releases the power on the motors
 **/
void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(robot.arms[i].motor_enable_pin,HIGH);
  }
}


/**
 * print the current motor positions in steps
 */
void motor_where() {
  output("0",robot.arms[0].new_step);
  output("1",robot.arms[1].new_step);
  output("2",robot.arms[2].new_step);
  output("3",robot.arms[3].new_step);
  output("4",robot.arms[4].new_step);
  output("5",robot.arms[5].new_step);
} 
