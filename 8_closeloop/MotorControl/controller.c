
#include "MyProject.h"


/****************************************************************************/
AUTOTURNING_t  autotuning_;
CONTROLLER_Config_t  ctrl_config;

uint32_t  control_error;
//float last_error_time_ = 0.0f;
// Inputs
float *controller_pos_estimate_linear_src_ = NULL;
float *controller_pos_estimate_circular_src_ = NULL;
float *controller_vel_estimate_src_ = NULL;
float *controller_pos_wrap_src_ = NULL; 

float pos_setpoint_ = 0.0f; // [turns]
float vel_setpoint_ = 0.0f; // [turn/s]

float input_pos_ = 0.0f;     // [turns]
float input_vel_ = 0.0f;     // [turn/s]
float input_torque_ = 0.0f;  // [Nm]
float input_filter_kp_ = 0.0f;
float input_filter_ki_ = 0.0f;

float autotuning_phase_ = 0.0f;

// bool input_pos_updated_ = false;

bool trajectory_done_ = true;

float mechanical_power_ = 0.0f; // [W]
float electrical_power_ = 0.0f; // [W]

// Outputs
float torque_output_ = 0.0f;
/****************************************************************************/
void Controller_update_filter_gains(void);  //函数声明
/****************************************************************************/
void controller_config_default(void)
{
	ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
	ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;  //see: InputMode_t
	ctrl_config.pos_gain = 20.0f;                  // [(turn/s) / turn]
	ctrl_config.vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
	ctrl_config.vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
	ctrl_config.vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
	ctrl_config.vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
	ctrl_config.vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
	ctrl_config.vel_ramp_rate = 1.0f;              // [(turn/s) / s]
	ctrl_config.torque_ramp_rate = 0.01f;          // Nm / sec
	ctrl_config.circular_setpoints = 0;
	ctrl_config.circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
	//ctrl_config.steps_per_circular_range = 1024;
	ctrl_config.inertia = 0.0f;                    // [Nm/(turn/s^2)]
	ctrl_config.input_filter_bandwidth = 2.0f;     // [1/s]
	ctrl_config.homing_speed = 0.25f;              // [turn/s]
	
	ctrl_config.gain_scheduling_width = 10.0f;
	ctrl_config.enable_gain_scheduling = false;
	ctrl_config.enable_vel_limit = true;
	ctrl_config.enable_overspeed_error = true;
	ctrl_config.enable_torque_mode_vel_limit = true;  // 力矩模式下，如果电机转速超过vel_limit ，电机输出的力矩将会减小
	//ctrl_config.axis_to_mirror = -1;
	//ctrl_config.mirror_ratio = 1.0f;
	//ctrl_config.torque_mirror_ratio = 0.0f;
	//ctrl_config.load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
	ctrl_config.mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
	ctrl_config.electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
	ctrl_config.spinout_electrical_power_threshold = 10.0f;  // [W] electrical power threshold for spinout detection
	ctrl_config.spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection
	
	Controller_update_filter_gains();
}
/**************************************/
void controller_para_init(void)
{
	ctrl_config.control_mode = CONTROL_mode;         //控制模式
	ctrl_config.input_mode = INPUT_mode;             //输入模式
	ctrl_config.torque_ramp_rate = TORQUE_ramp_rate; //Nm / sec，力矩爬升率
	ctrl_config.vel_ramp_rate = VELOCITY_ramp_rate;  //速度的爬升率
	ctrl_config.vel_gain = VELOCITY_P;               //速度P参数
	ctrl_config.vel_integrator_gain = VELOCITY_I;    //速度I参数
	ctrl_config.vel_limit = VELOCITY_limit;          //最大转速限制，圈/秒
	ctrl_config.pos_gain = POSITION_P;               //位置P参数
}
/****************************************************************************/
//arm() 函数中调用一次
void controller_reset(void) 
{
	vel_setpoint_ = 0.0f;
	mechanical_power_ = 0.0f;
	electrical_power_ = 0.0f;
}
/****************************************************************************/
void move_to_pos(float goal_point)
{
	planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_, trapTraj_config.vel_limit, trapTraj_config.accel_limit, trapTraj_config.decel_limit);
	trap_traj_.t_ = 0.0f;
	trajectory_done_ = false;
}
/****************************************************************************/

/****************************************************************************/
//生成的两个变量用于 INPUT_MODE_POS_FILTER 模式
void Controller_update_filter_gains(void)
{
	float bandwidth = min(ctrl_config.input_filter_bandwidth, 0.25f * current_meas_hz);
	input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
	input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}
/****************************************************************************/
static float limitVel(float vel_limit, float vel_estimate, float vel_gain, float torque)
{
	float Tmax = (vel_limit - vel_estimate) * vel_gain;
	float Tmin = (-vel_limit - vel_estimate) * vel_gain;
	return clamp(torque, Tmin, Tmax);
}
/****************************************************************************/
bool controller_update(void)
{
	float *pos_estimate_linear;
	float *pos_estimate_circular;
	float *pos_wrap;
	float *vel_estimate;
	float torque = 0.0f;
		
	vel_estimate = controller_vel_estimate_src_;
	
	if (!has_value(vel_estimate))
		return 0;
	torque = input_torque_; 
	// Never command a setpoint beyond its limit
	const float Tlim = motor_max_available_torque();   //max_torque = effective_current_lim_ * config_.torque_constant = 60*0.04f = 2.4f;
	torque = clamp(torque, -Tlim, Tlim);

  	float vel_gain = ctrl_config.vel_gain;
	float vel_integrator_gain = ctrl_config.vel_integrator_gain;	
	// Velocity limiting in current mode
	if (ctrl_config.control_mode < CONTROL_MODE_VELOCITY_CONTROL && ctrl_config.enable_torque_mode_vel_limit)
	{
		torque = limitVel(ctrl_config.vel_limit, *vel_estimate, vel_gain, torque);
	}

	// Torque limiting
	uint8_t limited = 0;
	if (torque > Tlim)
	{
		limited = 1;
		torque = Tlim;
	}
	if (torque < -Tlim)
	{
		limited = 1;
		torque = -Tlim;
	}
	
	torque_output_ = torque;

	motor_error &= ~ERROR_INVALID_ESTIMATE;
	
	return 1;
}
/****************************************************************************/



