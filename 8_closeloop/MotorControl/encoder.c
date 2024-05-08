
#include "MyProject.h"


/*****************************************************************************/
//GPIO1为CS
#define  SPI_CS0_L   GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define  SPI_CS0_H   GPIO_SetBits(GPIOA, GPIO_Pin_0)

#define  SPI3_TX_OFF() {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=0<<(12*2);}  //PC12(MOSI)输入浮空
#define  SPI3_TX_ON()  {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=2<<(12*2);}  //PC12(MOSI)复用推挽输出
/*****************************************************************************/
ENCODER_CONFIG   encoder_config;

static float pll_kp_ = 0.0f;   // [count/s / count]
static float pll_ki_ = 0.0f;   // [(count/s^2) / count]

bool pos_estimate_valid_ = false;   //位置估算是否可用
bool vel_estimate_valid_ = false;   //速度估算是否可用

float pos_estimate_ = 0.0f; //当前估算的位置值，单位[turn]
float vel_estimate_ = 0.0f; //当前估算转速，单位[turn/s]
float pos_circular_ = 0.0f; //环形位置模式下当前位置值，单位[turn]

/*****************************************************************************/
/*****************************************************************************/
void encoder_set_error(uint32_t error) 
{
	vel_estimate_valid_ = false;
	pos_estimate_valid_ = false;
	set_error(error);
}
/*****************************************************************************/
void update_pll_gains(void)
{
	pll_kp_ = 2.0f * encoder_config.bandwidth;  // basic conversion to discrete time
	pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped
	// Check that we don't get problems with discrete time approximation
	if (!(current_meas_period * pll_kp_ < 1.0f))
		encoder_set_error(ERROR_UNSTABLE_GAIN);
}
/*****************************************************************************/

/*****************************************************************************/
//初始化三种SPI接口的编码器的参数, 初始化I2C接口或者SPI接口
void MagneticSensor_Init(void)
{
	//读取flash内保存的参数
	encoder_config.mode = ENCODER_mode;  //选择编码器型号，参数设置宏定义在MyProject.h文件中
	encoder_config.cpr = ENCODER_cpr;    //编码器cpr
	encoder_config.bandwidth = ENCODER_bandwidth;   //编码器带宽
	encoder_config.calib_range = 0.02f; // Accuracy required to pass encoder cpr check  2%误差
	encoder_config.calib_scan_distance = 16.0f * M_PI; // rad electrical    校准的时候正反转8个极对数
	encoder_config.calib_scan_omega = 4.0f * M_PI;     // rad/s electrical  转速2个极对数/秒，所以正转4秒，反转再4秒
	encoder_config.phase_offset = 0;        // Offset between encoder count and rotor electrical phase
	encoder_config.phase_offset_float = 0.0f; // Sub-count phase alignment offset
	encoder_config.index_offset = 0.0f;
	encoder_config.use_index = false;
	encoder_config.pre_calibrated = false;
	encoder_config.direction = 0; // direction with respect to motor
	encoder_config.use_index_offset = true;
	encoder_config.enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
	encoder_config.find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
	
	update_pll_gains();    //锁相环参数整定
	
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL:
			TIM3_Encoder_Init();         //ABZ
			break;
		case MODE_SPI_AS5047P:
			SPI3_Init_(SPI_CPOL_Low);    //AS5047P
			break;
		case MODE_SPI_MT6701:
			SPI3_Init_(SPI_CPOL_Low);    //MT6701
			break;
		case MODE_SPI_MA730:
			SPI3_Init_(SPI_CPOL_High);   //MA730
			break;
		case MODE_SPI_TLE5012B:        //TLE5012B
			SPI3_Init_(SPI_CPOL_Low);
			break;
	}
}
/*****************************************************************************/
// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool run_offset_calibration(void)
{
	uint32_t  i;
	const float start_lock_duration = 1.0f;	
	encoder_config.direction = 1;
	encoder_config.phase_offset = 23406;
	encoder_config.phase_offset_float = 0.56f;
	return true;
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
	delta_enc = mod(delta_enc, encoder_config.cpr);
	if(delta_enc > encoder_config.cpr/2)
	delta_enc -= encoder_config.cpr;
	shadow_count_ += delta_enc;
	count_in_cpr_ = pos_abs_latched;
	
	// Memory for pos_circular
	float pos_cpr_counts_last = pos_cpr_counts_;
	
	// run pll (for now pll is in units of encoder counts)
	// Predict current pos
	pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
	pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
	
	// discrete phase detector
	float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
	float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(encoder_config.cpr));
	// pll feedback
	pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
	pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
	pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(encoder_config.cpr));
	vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
	uint8_t snap_to_zero_vel = false;
	if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
	{
		vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
		snap_to_zero_vel = true;
	}
	
	// Outputs from Encoder for Controller
	pos_estimate_ = pos_estimate_counts_ / (float)encoder_config.cpr;
	vel_estimate_ = vel_estimate_counts_ / (float)encoder_config.cpr;

	//// run encoder count interpolation
	int32_t corrected_enc = count_in_cpr_ - encoder_config.phase_offset;
	// if we are stopped, make sure we don't randomly drift
	if(snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
	{
		interpolation_ = 0.5f;
		// reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
	}
	else if(delta_enc > 0){
		interpolation_ = 0.0f;
	}else if(delta_enc < 0){
		interpolation_ = 1.0f;
	}
	else {
		// Interpolate (predict) between encoder counts using vel_estimate,
		interpolation_ += current_meas_period * vel_estimate_counts_;
		// don't allow interpolation indicated position outside of [enc, enc+1)
		if (interpolation_ > 1.0f) interpolation_ = 1.0f;
		if (interpolation_ < 0.0f) interpolation_ = 0.0f;
	}
	float interpolated_enc = corrected_enc + interpolation_;
	
	//// compute electrical phase
	//TODO avoid recomputing elec_rad_per_enc every time
	float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
	float ph = elec_rad_per_enc * (interpolated_enc - encoder_config.phase_offset_float);
	
	encoder_config.phase_ = wrap_pm_pi(ph) * encoder_config.direction;
	encoder_config.phase_vel_ = (2*M_PI) * vel_estimate_ * motor_config.pole_pairs * encoder_config.direction;

	return 1;
}
/****************************************************************************/



