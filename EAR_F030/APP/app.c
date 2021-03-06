

/**
********************************************************************************
* 版權：
* 模块名称：app.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/
#include "app.h"
#include "datatype.h"
#include "comm_task.h"
#include "CMD_Receive.h"
#include "serial_port.h"
#include "hardware.h"
#include "Motor_pwm.h"
#include "fifo.h"
#include "protocol_module.h"
#include "stm32f0xx_rtc.h"
#include "key_led_task.h"
//#include "exp_task.h" 
#include "stm32f0xx_usart.h"
//#include "store_fifo.h"
#include "delay.h"
#include "honeywell_sampling_data.h"

/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/
 // 命令接收控制对象
extern CMD_Receive g_CmdReceive; 

//發送數據FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern BOOL b_Is_PCB_PowerOn;

//保存數據
//extern FIFO_TYPE train_store_fifo;
//extern STORE_HEAD exp_store_head;
//extern TRAIN_STORE_DATA_PAGE train_store_data;
/***********************************
* 局部变量
***********************************/

/***********************************
* 函数声明
***********************************/
void test_task(void);
/***********************************
* 函数定义
***********************************/

//extern int16_t zero_point_of_pressure_sensor;


//初始化任务
void init_task(void)
{
	//初始化硬件
	init_hardware();	
	Motor_PWM_Init();
	
	//现在使用honeywell sensor，不需要校验了
	//Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
	
	//初始化通信相关
	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
	Init_Receive(&g_CmdReceive);
	//init_system(b_Is_PCB_PowerOn);

	os_create_task(TaskDataSend, OS_TRUE, SEND_TASK_ID);
	os_create_task(CMD_ProcessTask, OS_TRUE, RECEIVE_TASK_ID);
	
	#ifdef _DEBUG
//	os_create_task(key_led_task, OS_TRUE, KEY_LED_TASK_ID);
	os_create_task(honeywell_sampling_data,OS_TRUE,HONEYWELL_SAMPLING_DATA_TASK_ID);
	//os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	#else
	os_create_task(key_led_task, OS_TRUE, KEY_LED_TASK_ID);
	os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	os_create_task(get_switch_mode,OS_TRUE,TASK_GET_SWITCH_MODE);
	os_create_task(honeywell_sampling_data,OS_TRUE,HONEYWELL_SAMPLING_DATA_TASK_ID);
	os_create_task(led_show,OS_TRUE,TASK_LED_SHOW);
	
	//os_create_task(adc_value_sample,OS_TRUE,TASK_ADC_VALUE_SAMPLE);  //这个条不要了，换成honeywell sensor了
	#endif
	
	
	os_pend_task(INIT_TASK_ID);
}


void test_task(void)
{
	os_delay_ms(TEST_TASK_ID, 50);
}
