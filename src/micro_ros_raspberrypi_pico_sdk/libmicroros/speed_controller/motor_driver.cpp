#include "motor_driver/motor_driver.h"

void pwm_init_one_wheel(uint pin_a, uint pin_b, uint slice_num) {
  // Tell pin_a and pin_b they are allocated to the PWM
  gpio_set_function(pin_a, GPIO_FUNC_PWM);
  gpio_set_function(pin_b, GPIO_FUNC_PWM);

  // Set period of 5000 cycles (0 to 3 inclusive)
  // ==> fpwm = fc/5000  = 25kHz   (fc: 125Mhz, f_use: 25kHz )
  pwm_set_wrap(slice_num, WRAP);

  // Set the PWM running
  pwm_set_enabled(slice_num, true);
}

void pwm_init() {
  pwm_get_default_config();
  pwm_init_one_wheel(MotorFL_1.h_bridge.pwm_a, MotorFL_1.h_bridge.pwm_b, PWM1_SLICE);
  pwm_init_one_wheel(MotorRL_2.h_bridge.pwm_a, MotorRL_2.h_bridge.pwm_b, PWM2_SLICE);
  pwm_init_one_wheel(MotorFR_3.h_bridge.pwm_a, MotorFR_3.h_bridge.pwm_b, PWM3_SLICE);
  pwm_init_one_wheel(MotorRR_4.h_bridge.pwm_a, MotorRR_4.h_bridge.pwm_b, PWM4_SLICE);
}

void motor_run(uint motor_num, uint direct, float duty) {
  uint pin_a, pin_b, slice_num;
  switch (motor_num) {
    case 1:
      slice_num = PWM1_SLICE;
      break;
    case 2:
      slice_num = PWM2_SLICE;
      break;
    case 3:
      slice_num = PWM3_SLICE;
      break;
    case 4:
      slice_num = PWM4_SLICE;
      break;
    default:
      slice_num = PWM4_SLICE;
      break;
  }
  if (motor_num == 1 || motor_num == 2){
     if (direct == FORWARD) {
      pwm_set_chan_level(slice_num, PWM_CHAN_A, (WRAP + 1) * duty);
      pwm_set_chan_level(slice_num, PWM_CHAN_B, (WRAP + 1) * 0);
    }
    if (direct == REVERSE) {
      pwm_set_chan_level(slice_num, PWM_CHAN_B, (WRAP + 1) * duty);
      pwm_set_chan_level(slice_num, PWM_CHAN_A, (WRAP + 1) * 0);
    }
  }
 if (motor_num == 3 || motor_num == 4){
     if (direct == FORWARD) {
      pwm_set_chan_level(slice_num, PWM_CHAN_B, (WRAP + 1) * duty);
      pwm_set_chan_level(slice_num, PWM_CHAN_A, (WRAP + 1) * 0);
    }
    if (direct == REVERSE) {
      pwm_set_chan_level(slice_num, PWM_CHAN_A, (WRAP + 1) * duty);
      pwm_set_chan_level(slice_num, PWM_CHAN_B, (WRAP + 1) * 0);
    }
  }  
 
}
void left_motor_stop() {
  pwm_set_chan_level(PWM1_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM1_SLICE, PWM_CHAN_B, 0);
  pwm_set_chan_level(PWM2_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM2_SLICE, PWM_CHAN_B, 0);
}

void right_motor_stop() {
  pwm_set_chan_level(PWM3_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM3_SLICE, PWM_CHAN_B, 0);
  pwm_set_chan_level(PWM4_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM4_SLICE, PWM_CHAN_B, 0);
}

void all_motor_stop() {
  pwm_set_chan_level(PWM1_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM1_SLICE, PWM_CHAN_B, 0);
  pwm_set_chan_level(PWM2_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM2_SLICE, PWM_CHAN_B, 0);
  pwm_set_chan_level(PWM3_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM3_SLICE, PWM_CHAN_B, 0);
  pwm_set_chan_level(PWM4_SLICE, PWM_CHAN_A, 0);
  pwm_set_chan_level(PWM4_SLICE, PWM_CHAN_B, 0);
}