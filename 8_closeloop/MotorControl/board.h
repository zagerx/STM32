
#ifndef __BOARD_LIB_H
#define __BOARD_LIB_H

// #include "MyProject.h"

bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
void pwm_update_cb(void);
void foc_reset(void);
void motor_para_init(void);

float motor_max_available_torque(void);
void torqueMode_limitIq(float torque);


void arm(void);
void disarm(void);

#endif

