/**
 * @file encoder.h
 * @author Hien Nguyen
 * @brief Encoder module is used in the BamBooBot under minht57lab
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ENCODER__H
#define ENCODER__H

#include "motor_driver/motor_driver.h"
#include "quadrature_encoder.pio.h"

#define M1_SM 0
#define M2_SM 1
#define M3_SM 2
#define M4_SM 3

typedef struct {
  int cnt1;
  int cnt2;
  int cnt3;
  int cnt4;
} Cnt_t;

void encoder_init();
float read_enc(uint sm, int *cnt_k1);
void reset_cnt(Cnt_t *cnt_t);

#endif /* ENCODER__H */