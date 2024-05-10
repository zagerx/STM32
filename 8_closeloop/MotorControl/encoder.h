
#ifndef __ENCODER_LIB_H
#define __ENCODER_LIB_H

extern  float  pos_estimate_;
extern  float  vel_estimate_;
extern  float  encoder_elespeed;
extern  float  encoder_eletheta;
/****************************************************************************/
void MagneticSensor_Init(void);
bool encoder_update(void);
/****************************************************************************/

#endif

