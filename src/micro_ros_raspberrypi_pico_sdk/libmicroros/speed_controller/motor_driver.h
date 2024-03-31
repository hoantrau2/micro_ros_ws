/**
 * @file motor_driver.h
 * @author Hien Nguyen
 * @brief Motor_driver module is used in the BamBooBot under minht57lab
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MOTOR_DRIVER__H
#define MOTOR_DRIVER__H

#include "config/config.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

using namespace bbb::config;

#define WRAP 4999 // fpwm = fc/5000 (fc 125Mhz) = 25kHz
#define FORWARD 0
#define REVERSE 1

// Find out which PWM slice is connected to PWMx_PIN
const uint PWM1_SLICE = pwm_gpio_to_slice_num(MotorFL_1.h_bridge.pwm_a);
const uint PWM2_SLICE = pwm_gpio_to_slice_num(MotorRL_2.h_bridge.pwm_a);
const uint PWM3_SLICE = pwm_gpio_to_slice_num(MotorFR_3.h_bridge.pwm_a);
const uint PWM4_SLICE = pwm_gpio_to_slice_num(MotorRR_4.h_bridge.pwm_a);

void pwm_init_one_wheel(uint pin_a, uint pin_b, uint slice_num);

void pwm_init();
void motor_run(uint motor_num, uint direct, float duty); // duty 0->1
void left_motor_stop();
void right_motor_stop();
void all_motor_stop();

#endif /* MOTOR_DRIVER__H */
