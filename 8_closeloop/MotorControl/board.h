
#ifndef __BOARD_LIB_H
#define __BOARD_LIB_H

#include "MyProject.h"


/****************************************************************************/
extern  Iph_ABC_t  current0;
extern  float  vbus_voltage;
/****************************************************************************/
/****************************************************************************/

/****************************************************************************/
bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
void pwm_update_cb(void);
void foc_reset(void);


/****************************************************************************/
void motor_para_init(void);
void arm(void);
void disarm(void);

float motor_max_available_torque(void);
void torqueMode_limitIq(float torque);
/****************************************************************************/

#endif

