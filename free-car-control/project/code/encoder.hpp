#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include "zf_common_typedef.hpp"
#include <stdio.h>

#define model_curve_same_integral 3000
#define model_curve_opposite_integral 3000
#define model_non_curve_integral 1000


#define model_bypass_stage3_integral 2000

#define model_straight_reopen_integral 6000
#define model_protect_encoder_integral 10000

extern float f_encoder_left,f_encoder_right;
extern float encoder_left_abs_total;
extern float encoder_right_abs_total;
extern float encoder_left_sum_total;
extern float encoder_right_sum_total;
extern float model_encoder_integral;

void encoder_clear(void);
void encoder_distance_reset_total(void);
void filter_encoder_get(float filter_alpha);
void model_encoder_increment(void);
void model_encoder_clear_increment(void);


#endif
