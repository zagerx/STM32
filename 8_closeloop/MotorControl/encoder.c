#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "utils.h"

/*****************************************************************************/
//GPIO1为CS
#define  SPI_CS0_L   GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define  SPI_CS0_H   GPIO_SetBits(GPIOA, GPIO_Pin_0)

#define  SPI3_TX_OFF() {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=0<<(12*2);}  //PC12(MOSI)输入浮空
#define  SPI3_TX_ON()  {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=2<<(12*2);}  //PC12(MOSI)复用推挽输出
/*****************************************************************************/

static float pll_kp_ = 2000.0f;      // [count/s / count]
static float pll_ki_ = 1000000.0f;   // [(count/s^2) / count]
float vel_estimate_ = 0.0f;          //当前估算转速，单位[turn/s]
float pos_estimate_ = 0.0f;          //当前估算的位置值，单位[turn]
float encoder_elespeed = 0.0f;
float encoder_eletheta = 0.0f;
#define ENCODER_DIR             1
#define ENCODER_OFFSET          23406
#define ENCODER_OFFSET_FLOAT    0.56f
#define ENCODER_PERIOD          0.000125f
#define ENCODER_MOTOR_PAIR      7.0f
#define  ENCODER_CPR                   16384     //AS5047P=MT6701=16384,TLE5012B=32768, MODE_INCREMENTAL=4000,
//初始化三种SPI接口的编码器的参数, 初始化I2C接口或者SPI接口
void MagneticSensor_Init(void)
{
	SPI3_Init_(SPI_CPOL_Low);    //MT6701
}

/*****************************************************************************/
/*****************************************************************************/
uint8_t ams_parity(uint16_t v)
{
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}
/*****************************************************************************/
uint16_t abs_spi_cb(void)
{
	uint16_t rawVal;
	uint16_t pos;
	
	SPI_CS0_L;
	rawVal = SPIx_ReadWriteByte(0xffff);  //encoder.hpp 第144行
	SPI_CS0_H;

	if(ams_parity(rawVal) || ((rawVal >> 14) & 1))
		return 0xFFFF;
	pos = (rawVal & 0x3fff);

	return pos;
}

/****************************************************************************/
static inline float wrap_pm_pi(float x) {
	return wrap_pm(x, 2 * M_PI);
}
bool encoder_update(void)
{
	int32_t delta_enc = 0;
	int32_t pos_abs_latched; //LATCH
	static int32_t shadow_count_ = 0;   //编码器累计计数。
	int32_t count_in_cpr_ = 0;   //编码器当前计数值。
	float interpolation_ = 0.0f; //编码器当前插补值。
	static float pos_estimate_counts_ = 0.0f;  //当前估算的位置值，单位[count]   
	static float pos_cpr_counts_ = 0.0f;       //当前约束在cpr范围内的位置值，单位[count]
	static float vel_estimate_counts_ = 0.0f;  //当前估算转速，单位[count/s]

	pos_abs_latched = abs_spi_cb();

	delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
	delta_enc = mod(delta_enc, ENCODER_CPR);
	if(delta_enc > ENCODER_CPR/2)
	delta_enc -= ENCODER_CPR;
	shadow_count_ += delta_enc;
	count_in_cpr_ = pos_abs_latched;
	
	// Memory for pos_circular
	float pos_cpr_counts_last = pos_cpr_counts_;
	
	// run pll (for now pll is in units of encoder counts)
	// Predict current pos
	pos_estimate_counts_ += ENCODER_PERIOD * vel_estimate_counts_;
	pos_cpr_counts_      += ENCODER_PERIOD * vel_estimate_counts_;
	
	// discrete phase detector
	float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
	float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(ENCODER_CPR));
	// pll feedback
	pos_estimate_counts_ += ENCODER_PERIOD * pll_kp_ * delta_pos_counts;
	pos_cpr_counts_ += ENCODER_PERIOD * pll_kp_ * delta_pos_cpr_counts;
	pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(ENCODER_CPR));
	vel_estimate_counts_ += ENCODER_PERIOD * pll_ki_ * delta_pos_cpr_counts;
	uint8_t snap_to_zero_vel = false;
	if (fabsf(vel_estimate_counts_) < 0.5f * ENCODER_PERIOD * pll_ki_)
	{
		vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
		snap_to_zero_vel = true;
	}
	
	// Outputs from Encoder for Controller
	pos_estimate_ = pos_estimate_counts_ / (float)ENCODER_CPR;
	vel_estimate_ = vel_estimate_counts_ / (float)ENCODER_CPR;

	//// run encoder count interpolation
	int32_t corrected_enc = count_in_cpr_ - ENCODER_OFFSET;
	// if we are stopped, make sure we don't randomly drift

	float interpolated_enc = corrected_enc + interpolation_;
	
	//// compute electrical phase
	//TODO avoid recomputing elec_rad_per_enc every time
	float elec_rad_per_enc = ENCODER_MOTOR_PAIR * 2 * M_PI * (1.0f / (float)(ENCODER_CPR));
	float ph = elec_rad_per_enc * (interpolated_enc - ENCODER_OFFSET_FLOAT);
	
	encoder_eletheta = wrap_pm_pi(ph) * ENCODER_DIR;
	encoder_elespeed = (2*M_PI) * vel_estimate_ * ENCODER_MOTOR_PAIR * ENCODER_DIR;

	return 1;
}
/****************************************************************************/



