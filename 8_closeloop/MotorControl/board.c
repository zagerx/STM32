
#include "MyProject.h"

typedef struct 
{
	float phA;
	float phB;
	float phC;
} Iph_ABC_t;

typedef struct 
{
	float d;
	float q;
} float2D;

#define  CURRENT_SENSE_MIN_VOLT  0.3f
#define  CURRENT_SENSE_MAX_VOLT  3.0f
#define  CURRENT_ADC_LOWER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f)
#define  CURRENT_ADC_UPPER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f)

static float  vbus_voltage=12.0f;
static Iph_ABC_t  current0;
static  Iph_ABC_t  current_meas_;//ToDo
static float  Ialpha_beta[2];
static float2D  motor_Idq_setpoint_;

static float  pi_gains_[2];
static float  v_current_control_integral_d = 0.0f; // [V]
static float  v_current_control_integral_q = 0.0f; // [V]
static float effective_current_lim_ = 10.0f; // [A]
static float max_allowed_current_ = 0.0f;    // [A] set in setup()


float input_torque_ = 0.0f;  // [Nm]

void arm(void);
void disarm(void);

#define enable_torque_mode_vel_limit (1)
#define TORQUE_COMSTANT     0.04F
#define  VELOCITY_P                      0.02f              //速度P参数
#define  VELOCITY_I                      0.2f               //速度I参数
#define  VELOCITY_limit                  50                 //最大速度限制，力矩模式超过限速力矩会下降


static float limitVel(float vel_limit, float vel_estimate, float vel_gain, float torque)
{
	float Tmax = (vel_limit - vel_estimate) * vel_gain;
	float Tmin = (-vel_limit - vel_estimate) * vel_gain;
	return clamp(torque, Tmin, Tmax);
}

static float torqueMode_limittorque(void)
{
	float *pos_estimate_linear;
	float *pos_estimate_circular;
	float *pos_wrap;
	float *vel_estimate;
	float torque = 0.0f;
		
	vel_estimate = &vel_estimate_;
	
	if (!has_value(vel_estimate))
		return 0;
	torque = input_torque_; 
	// Never command a setpoint beyond its limit
	const float Tlim = motor_max_available_torque();   //max_torque = effective_current_lim_ * config_.torque_constant = 60*0.04f = 2.4f;
	torque = clamp(torque, -Tlim, Tlim);

  	float vel_gain = VELOCITY_P;
	// Velocity limiting in current mode
	{
		torque = limitVel(VELOCITY_limit, *vel_estimate, vel_gain, torque);
	}

	// Torque limiting
	if (torque > Tlim)
	{
		torque = Tlim;
	}
	if (torque < -Tlim)
	{
		torque = -Tlim;
	}
	return torque;
}




 void motor_para_init(void)
{
	pi_gains_[0] = 0.0273f;
	pi_gains_[1] = 166.2507f;
	effective_current_lim_ = 30.0f;
	max_allowed_current_ = 60.75f;	
}





 void arm(void)
{
	foc_reset();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);   //使能输出
}

 void disarm(void)
{
	TIM_CtrlPWMOutputs(TIM1, DISABLE);  //停止输出
}


static float motor_max_available_torque(void)
{
	return 30.0f * TORQUE_COMSTANT;
}

static float phase_current_from_adcval(uint32_t ADCValue)
{
	if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND)
	{
		return 0;
	}
	
	int adcval_bal = (int)ADCValue - (1 << 11);
	float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
	float shunt_volt = amp_out_volt * (1/PHASE_CURRENT_GAIN);   //phase_current_rev_gain_ = 1/20倍放大
	float current = shunt_volt * (1/SHUNT_RESISTANCE);          //shunt_conductance_ = 1/0.001采样电阻;
	return current;
}
static void vbus_sense_adc_cb(uint32_t adc_value) 
{
	float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / 4096;
	vbus_voltage = adc_value * voltage_scale;
}
//在TIM1的更新中断函数中被调用
static uint8_t fetch_and_reset_adcs(Iph_ABC_t *current)
{
	uint8_t all_adcs_done = (((ADC1->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC2->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC3->SR & ADC_SR_JEOC) == ADC_SR_JEOC));
  if(!all_adcs_done)return 0;
	
	vbus_sense_adc_cb(ADC1->JDR1);
	current->phB = phase_current_from_adcval(ADC2->JDR1);
	current->phC = phase_current_from_adcval(ADC3->JDR1);
	current->phA = -current->phB - current->phC;
	current_meas_.phA = current->phA - 0.0f;//DC_calib_.phA;
	current_meas_.phB = current->phB - (-0.2f);//DC_calib_.phB;
	current_meas_.phC = current->phC - 0.2f;//DC_calib_.phC;	
	ADC1->SR = ~(ADC_SR_JEOC);
	ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	
	return 1;
}
/****************************************************************************
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
uint32_t time_cntr=0;

	if(++time_cntr >= 8000)  //0.5s
	{
		time_cntr = 0;
		LED_blink;
	}
****************************************************************************/
//中断频率16KHz，进入中断的同时触发ADC



static void torqueMode_limitIq(float torque)
{

	 torque = (1) * torque;
	
	// Load setpoints from previous iteration.
	float id = motor_Idq_setpoint_.d;
	float iq = motor_Idq_setpoint_.q;
	
	float ilim = 30.0f;
	
	id = clamp(id, -ilim*0.99f, ilim*0.99f); // 1% space reserved for Iq to avoid numerical issues
	iq = torque / TORQUE_COMSTANT;
	
	// 2-norm clamping where Id takes priority
	float iq_lim_sqr = SQ(ilim) - SQ(id);
	float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
	iq = clamp(iq, -Iq_lim, Iq_lim);
	

	motor_Idq_setpoint_.d = id;
	motor_Idq_setpoint_.q = iq;
	
}


/*****************************************************************************/
/****************************************************************************/
int SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

/*****************************************************************************/
unsigned short test_A,test_B,test_C;
static bool enqueue_modulation_timings(float mod_alpha, float mod_beta)
{
	float tA, tB, tC;
	if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
	{
		return false;
	}
	TIM1->CCR1 = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR2 = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR3 = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);

	test_A = TIM1->CCR1;
	test_B = TIM1->CCR2;
	test_C = TIM1->CCR3;
	return true;
}

/*****************************************************************************/
static void foc_reset(void)
{
	v_current_control_integral_d = 0.0f;  //积分清零，否则闭环时电机误动作
	v_current_control_integral_q = 0.0f;
	Ialpha_beta[0] = 0;
	Ialpha_beta[1] = 0;
}
/*****************************************************************************/
//0.5.6版本，clark变换与之后的变换分开处理

static bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase)   //I_phase是park变换时的θ，pwm_phase是park逆变换时的θ
{
	Ialpha_beta[0] = current_meas_.phA;
	Ialpha_beta[1] = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);


	// Park transform
	float c_I = our_arm_cos_f32(I_phase);
	float s_I = our_arm_sin_f32(I_phase);
	float Id = c_I * Ialpha_beta[0] + s_I * Ialpha_beta[1];
	float Iq = c_I * Ialpha_beta[1] - s_I * Ialpha_beta[0];
	// Current error
	float Ierr_d = Id_des - Id;
	float Ierr_q = Iq_des - Iq;
	
	// TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
	// Apply PI control
	float Vd = v_current_control_integral_d + Ierr_d * pi_gains_[0];
	float Vq = v_current_control_integral_q + Ierr_q * pi_gains_[0];
	
	float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
	float V_to_mod = 1.0f / mod_to_V;
	float mod_d = V_to_mod * Vd;
	float mod_q = V_to_mod * Vq;
	
	// Vector modulation saturation, lock integrator if saturated
	// TODO make maximum modulation configurable
	float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
	if (mod_scalefactor < 1.0f)
	{
		mod_d *= mod_scalefactor;
		mod_q *= mod_scalefactor;
		// TODO make decayfactor configurable
		v_current_control_integral_d *= 0.99f;
		v_current_control_integral_q *= 0.99f;
	}
	else
	{
		v_current_control_integral_d += Ierr_d * (pi_gains_[1] * current_meas_period);
		v_current_control_integral_q += Ierr_q * (pi_gains_[1] * current_meas_period);
	}
	
	// Inverse park transform
	float c_p = our_arm_cos_f32(pwm_phase);
	float s_p = our_arm_sin_f32(pwm_phase);
	float mod_alpha = c_p * mod_d - s_p * mod_q;
	float mod_beta = c_p * mod_q + s_p * mod_d;
	
	// Apply SVM
	if (!enqueue_modulation_timings(mod_alpha, mod_beta))
		return false; // error set inside enqueue_modulation_timings
	
	return true;
}
void pwm_update_cb(void)
{
	float pwm_phase = encoder_eletheta + 1.5f * current_meas_period * encoder_elespeed;  //0.5.6是在反park变换时计算，效果一样
	FOC_current(motor_Idq_setpoint_.d, motor_Idq_setpoint_.q, encoder_eletheta, pwm_phase);  //highcurrent电机用电流模式		
}
void TIM1_UP_TIM10_IRQHandler(void)
{
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	
	uint8_t counting_down = TIM1->CR1 & TIM_CR1_DIR;
	if(!counting_down)   //=0为递增计数,上臂为低下臂为高,此时采样
	{
		encoder_update();
		fetch_and_reset_adcs(&current0);     //电流采样，获得的采样值在current0
	}
	else
	{
		torqueMode_limitIq(torqueMode_limittorque());//更新iq限制值
		pwm_update_cb();         
	}
}



