
#include "MyProject.h"


/*****************************************************************************
代码演示 闭环控制
使用教程：https://blog.csdn.net/loop222/article/details/133888495
        《ODrive解析移植（八）—— 闭环控制》
创建日期：20230807
作    者：loop222 @郑州
****************************************************************************/
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
/****************************************************************************/
uint32_t timecntr_pre=0;

uint8_t usb_recbuff[128];
uint8_t usb_sndbuff[128];
uint32_t usb_rcv_count;
 extern  float input_vel_;
 extern  float input_pos_;
 extern  float input_torque_;
/******************************************************************************/
void commander_run(void);
void USBcommander_run(void);
/******************************************************************************/
//us计时，每71.5分钟溢出循环一次
uint32_t timecount(void)
{
	uint32_t  diff,now_us;
	
	now_us = micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;
	
	return diff;
}
/****************************************************************************/
uint32_t print_flag;

/****************************************************************************/
void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIO
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOD,GPIOC,GPIOB,GPIOA;AFIO;
	
	//指示灯为PD2，低电平灯亮，高电平灯灭，
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //IO输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}
/*****************************************************************************/
//本示例支持USB通信和USART2通信，
//USART2接线：GPIO3接USB转串口的RXD，GPIO4接USB转串口的TXD，GND共地
/*****************************************************************************/

typedef enum
{
	AXIS_STATE_IDLE = 0,
	AXIS_STATE_MOTOR_CALIBRATION,
	AXIS_STATE_ENCODER_INDEX_SEARCH,
	AXIS_STATE_ENCODER_DIR_FIND,
	AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
	AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
	AXIS_STATE_LOCKIN_SPIN,
	AXIS_STATE_CLOSED_LOOP_CONTROL,
	AXIS_STATE_UNDEFINED,
} AXIS_State_Type;
AXIS_State_Type  current_state_;

void run_state_machine_loop(void)
{
	bool status=0;
	uint32_t  len;
	switch(current_state_)
	{
		case AXIS_STATE_MOTOR_CALIBRATION:{
			current_state_ = AXIS_STATE_UNDEFINED;

		} break;
		
		case AXIS_STATE_CLOSED_LOOP_CONTROL: {

			arm();
			current_state_ = AXIS_STATE_IDLE;
		} break;
		
		case AXIS_STATE_IDLE: {
		} break;
		
		case AXIS_STATE_UNDEFINED: {
		} break;
		
		default: 
		break;
	}
	
	// If the state failed, go to idle, else advance task chain
	if (!status)
	{
		current_state_ = AXIS_STATE_IDLE;
	}
}


int main(void)
{
	GPIO_Configure();
	USART2_Init(115200);        //排针的GPIO3为TXD2，GPIO4为RXD2，GND共地
	tim14_InitTick();           //1ms中断初始化，为系统提供计时
	TIM1_PWM_Init();            //M0接口PWM配置，但没有使能
	ADC_Common_Init();          //初始化ADC的引脚和通用配置，设置中断
	ADC1_DMA_Init();            //配置注入组，检测vbus，等待TIM1触发
	ADC2_TRGO_Init();           //配置注入组，检测m0_phB，等待TIM1触发
	ADC3_TRGO_Init();           //配置注入组，检测m0_phC，等待TIM1触发
	TIM2_Init();                //触发ADC1的规则转换
	
	//参数设置在MyProject.h中，更多参数设置请进入以下几个函数中设置
	MagneticSensor_Init();

	motor_para_init();          //电机参数的设置

	
	delay_us(500000);
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc, &USBD_CDC_cb, &USR_cb);  //USB转串口,COM174
	
	TIM_Cmd(TIM1, ENABLE);           //TIM1触发中断，AD开始转换。但是PWM还没有输出

	USART2_SendDMA(sprintf(snd2_buff,"Motor Ready!\r\n"));
	
	//在MyProject.h中设置电机参数和控制模式。参数设置与官方代码一致，请先熟悉官方odrivetool的操作
	//发送指令“C”，测量电机的电阻电感并校准电机，发送指令“G”进入闭环，然后设置对应的速度或者位置

	while(1)
	{
		run_state_machine_loop();
		delay_us(1000);     //1ms，延时增加通信的可靠性
	}
}

/*****************************************************************************/
//USB虚拟串口通信
//放在USB接收中断里处理 App/usbd_cdc_vcp.c文件中的VCP_DataRx()函数中
void USBcommander_run(void)
{
	uint32_t len;
	
	if(usb_rcv_count != 0)
	{
		switch(usb_recbuff[0])
		{
			case 'H':
				usb_send(usb_sndbuff, len);
				break;
				
			case 'C': {  //测量电阻电感，3秒后电机“嘀”一声
				current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
			} break;

			case 'G': {  //闭环控制
				current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
			} break;
			case 'I': {  //空闲模式

				input_torque_ = 0;
				disarm();
				current_state_ = AXIS_STATE_IDLE;
			} break;
			
			case 'S': {  //设置力矩 S0.1
				input_torque_ = atof((const char *)(usb_recbuff+1));
				len=sprintf((char *)usb_sndbuff,"S=%.2f\r\n", input_torque_);
				usb_send(usb_sndbuff, len);
			} break;

			case 'V': {  //读取当前速度 turn/s
				len=sprintf((char *)usb_sndbuff, "vel=%.4f\r\n", vel_estimate_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'P': {  //读取当前位置 turn
				len=sprintf((char *)usb_sndbuff, "pos=%.4f\r\n", pos_estimate_);
				usb_send(usb_sndbuff, len);
			} break;
			case 'Q': {  //读取错误标志
//				len=sprintf((char *)usb_sndbuff, "error=%X\r\n", motor_error);
//				usb_send(usb_sndbuff, len);
			} break;
		}
		
		memset(usb_recbuff,0,usb_rcv_count); //清空接收数组
		usb_rcv_count = 0;
	}
}
/*****************************************************************************/


