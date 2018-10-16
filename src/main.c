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
int digit = 0;
int A = 8192*30/360;                              //angle between two target

//These are the parameters of PID controller
const float chassis_kp = 0;     //Proportional
const float chassis_ki = 0;     //Integration
const float chassis_kd = 0;     //Derivative

static int16_t set_angle(){
  if(digit == 1) return A;
  else if(digit == 3) return -A;
  else return 0;
}

static int16_t angle(const int16_t current, int16_t *previous) {
  int16_t change = 0;
  static int16_t total_angle_change = 0;

  change = current - *previous;

  if(change > 4095) change -= 4095;

  total_angle_change += change;
  *previous_angle = current_angle;

  return total_angle_change;
}

static int16_t pid_control(const int16_t setPoint, const int16_t current,
                           float* error_int, float* error_der,
                           int16_t* previous_error) {
  int16_t output = 0;
  int16_t error = 0;

  const int dt = 2;
  const int n = 19;

  error = setPoint*n - current;
  *error_int += error * dt;
  if (setPoint != 0) *error_int *= 0.985;

  *error_der = (error - *previous_error) / dt;

  output = chassis_kp * error + chassis_ki * *error_int
      + chassis_kd * *error_der; //Proportional controller

      //NOTE: Limit your maximum integrated error is often useful
  float MAX_ERROR_INT = 10000;

  if (*error_int > MAX_ERROR_INT)
    *error_int = MAX_ERROR_INT;
  else if (*error_int < -MAX_ERROR_INT)
    *error_int = -MAX_ERROR_INT;

  chThdSleepMilliseconds(dt);

  *previous_error = error;

  if (output > 10000)
    output = 10000;
  else if (output < -10000)
    output = -10000;

  return output;
}

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p) {

  (void)p;

  Encoder_canStruct* encoder = can_getEncoder(); //Pointer to motor encoder feedback


  static float motor_error_int; //error integrators for the four motors
  static float motor_error_der; //error derivatives for the four motors
  static int16_t previous_error;
  static int16_t previous_angle;

  previous_angle = (encoder + i)->angle_rotor_raw;

  while (true) {

    int16_t current_angle = angle((encoder + i)->angle_rotor_raw,
                                  previous_angle);
    motor_output = pid_control(set_angle(),
                               current_angle,
                               motor_error_int,
                               motor_error_der,
                               previous_error);
    /*
     TODO set motor current
     */
    can_motorSetCurrent(0x200, motor_output, 0, 0, 0);

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
    palTogglePad(GPIOA, GPIOA_LED);
    chThdSleepMilliseconds(500);
  }
}
