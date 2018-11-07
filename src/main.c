/*
 ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"

#include <stdlib.h>

static RC_Ctl_t* rc;

static int16_t motor_output;        //Torque command for motors
static int digit = 0, predigit = 0;
const float pi = 3.14159265358;
double A;                              //angle between two target

//These are the parameters of PID controller
float chassis_kp = 100;     //Proportional
const float chassis_ki = 0.5;     //Integration
const float chassis_kd = 1500;     //Derivative

static float set_angle(int x){
  if(x == 1) return A;
  else if(x == 3) return -A;
  else return 0;
}

static int16_t pid_control(const float setPoint, const float current,
                           float* error_int, float* error_der,
                           int16_t* previous_error) {
  int16_t output = 0;

  int16_t error = 0;


  error = setPoint - current;
  *error_int += error;
  *error_int *= 0.5;

  *error_der = (error - *previous_error);

  if(error <= 19*pi/3 && error >= -19*pi/3) chassis_kp = 150;
  else chassis_kp = 100;

  output = chassis_kp * error + chassis_ki * *error_int
      + chassis_kd * *error_der; //Proportional controller

      //NOTE: Limit your maximum integrated error is often useful
  float MAX_ERROR_INT = 10000000000;

  if (*error_int > MAX_ERROR_INT)
    *error_int = MAX_ERROR_INT;
  else if (*error_int < -MAX_ERROR_INT)
    *error_int = -MAX_ERROR_INT;

  chThdSleepMilliseconds(2);

  *previous_error = error;

  if (output > 5000)
    output = 5000;
  else if (output < -5000)
    output = -5000;

  return output;
}

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p) {

  (void)p;

  Encoder_canStruct* encoder = can_getEncoder(); //Pointer to motor encoder feedback

  A = pi/3;

  static float motor_error_int; //error integrators for the four motors
  static float motor_error_der; //error derivatives for the four motors
  static int16_t previous_error;

  float initial_angle;
  initial_angle = (encoder)->radian_angle;
  chThdSleepMilliseconds(2);

  while (true) {
    if(rc->s2 == 1) digit = 1;
    else if(rc->s2 == 3) digit = 2;
    else digit = 3;

    if(rc -> channel3 < 370) digit = 0;


      motor_output = pid_control(19*(set_angle(digit)+initial_angle),
                                  (encoder)->radian_angle,
                                   &motor_error_int,
                                   &motor_error_der,
                                   &previous_error);

      can_motorSetCurrent(0x200, motor_output, 0, 0, 0);

      if((digit == 1 || digit == 2 || digit == 3)
          && (encoder)->radian_angle >= 19*(set_angle(digit)+initial_angle-A/4)
          && (encoder)->radian_angle <= 19*(set_angle(digit)+initial_angle+A/4)
          && (encoder)->speed_rpm == 0)
      {

        if(digit != predigit){
            palSetPad(GPIOA,2);
            chThdSleepMilliseconds(500);

            palClearPad(GPIOA, 2);
            chThdSleepMilliseconds(500);

            predigit = digit;
         }
      }
      else if(digit == 0) predigit = 0;


    chThdSleepMilliseconds(2);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  RC_init();
  can_processInit();

  rc = RC_get();

  chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
                    NORMALPRIO,motor_ctrl_thread, NULL);

  /*
   * Normal main() thread activity
   */

  while (true) {

    palClearPad(GPIOA,2);

    if(set_angle(digit) == -A) palClearPad(GPIOA, GPIOA_LED);

    else if(set_angle(digit) == 0) palSetPad(GPIOA, GPIOA_LED);

    else if(set_angle(digit) == A) palTogglePad(GPIOA, GPIOA_LED);

    chThdSleepMilliseconds(500);
  }
}
