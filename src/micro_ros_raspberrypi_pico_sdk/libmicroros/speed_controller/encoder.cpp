#include "encoder/encoder.h"

PIO pio = pio0;

void encoder_init() {
  uint offset = pio_add_program(pio, &quadrature_encoder_program);

  quadrature_encoder_program_init(pio, M1_SM, offset, MotorFL_1.encoder.cha, 0);

  quadrature_encoder_program_init(pio, M2_SM, offset, MotorRL_2.encoder.cha, 0);

  quadrature_encoder_program_init(pio, M3_SM, offset, MotorFR_3.encoder.cha, 0);

  quadrature_encoder_program_init(pio, M4_SM, offset, MotorRR_4.encoder.cha, 0);
}

float read_enc(uint sm, int *cntk_1) {
  int cnt_k;
  float w_speed;
  cnt_k = quadrature_encoder_get_count(pio, sm);
  w_speed =
    ((cnt_k - *cntk_1) * 2 * 3.14) / (SAMPLE_TIME * 1e-3 * MotorFL_1.encoder.pules_one_round); // rad/s
  // w_speed = ((cnt_k - cnt_k1) * 60) /
  //           (SAMPLE_TIME * 1e-3 * PPR); // rpm (round/minute)
  if (sm == M1_SM || sm == M2_SM) {
    w_speed *= (-1);
  }
  *cntk_1 = cnt_k;

  return (w_speed * Bbb_parameter.wheel_diameter); // m/s
  // return (w_speed); // rad/s
}

void reset_cnt(Cnt_t *cnt_t) {
  (cnt_t->cnt1) = 0.0;
  (cnt_t->cnt2) = 0.0;
  (cnt_t->cnt3) = 0.0;
  (cnt_t->cnt4) = 0.0;
}
