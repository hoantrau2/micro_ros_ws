/**
 * @file velocity_convertor.h
 * @author Hien Nguyen
 * @brief Velocity_convertor module is used in the BamBooBot under minht57lab
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef VELOCCITY__H
#define VELOCCITY__H

#include <stdlib.h>
#include "encoder/encoder.h"
#include "motor_driver/motor_driver.h"
#include "speed_controller/speed_controller.h"
#include "uart_display/uart_display.h"

#define V_MAX 1.15 // m/s <=> 70% duty
#define V_MIN 0 // m/s 
#define sgn(x) ((x) < 0 ? -1 : 1)
#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4

typedef struct {
  PID_t PID_M1_t;
  PID_t PID_M2_t;
  PID_t PID_M3_t;
  PID_t PID_M4_t;
} PIDs_t;

typedef struct {
  PIDs_t pids_t;
  Cnt_t cnt_t;
  double v;
  double w;
} Event_motor_t;

void control_motor(void *cxt);

#endif /*VELOCCITY__H*/