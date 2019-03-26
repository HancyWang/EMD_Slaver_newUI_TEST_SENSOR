

/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�key_led_task.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
***********************************/

#include "app.h"
#include "datatype.h"
#include "hardware.h"
#include "fifo.h"
#include "key_led_task.h"
#include "protocol_module.h"

#include "i2c.h"
#include "Motor_pwm.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_dma.h"
#include "serial_port.h"
#include "CMD_receive.h"
#include "app.h"
#include "delay.h"
#include "comm_task.h"
#include "iwtdg.h"
#include "honeywell_sampling_data.h"
/**********************************
*�궨��
***********************************/
#define BATTERY_NO_POWER_THRESHOLD 		3276  //3345��Ӧ2.45V  3276 2.4v
#define BATTERY_LOW_POWER_THRESHOLD		3549  //3549��Ӧ2.6V
/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* �ֲ�����
***********************************/
extern HONEYWELL_STATE honeywell_state;

extern uint32_t os_ticks;
extern CMD_Receive g_CmdReceive;  // ������տ��ƶ���

extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];
extern CHCKMODE_OUTPUT_PWM state;

extern PWM_STATE pwm1_state;
extern PWM_STATE pwm2_state;
extern PWM_STATE pwm3_state;

extern uint8_t led_high_cnt;
extern uint8_t led_low_cnt;
extern uint8_t flash_cnt;
//KEYֵ������㰴Ϊȷ����������
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;

MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;

//extern uint8_t OUTPUT_FINISH;
BOOL b_Is_PCB_PowerOn=FALSE;
//BOOL b_check_bat=FALSE;
volatile KEY_STATE key_state=KEY_UPING;

extern uint16_t RegularConvData_Tab[2];
extern uint8_t adc_state;
//extern THERMAL_STATE thermal_state;

static uint8_t wakeup_Cnt;

LED_STATE led_state=LED_NONE;

extern uint8_t prev_mode;

extern BOOL b_getHoneywellZeroPoint;
extern uint32_t adc_pressure_value;
extern uint32_t HONEYWELL_ZERO_POINT;

BOOL b_check_BAT_ok=FALSE;

extern uint8_t switch_mode_cnt;
extern BOOL b_check_bnt_release;
extern uint8_t release_btn_cnt;

extern uint16_t checkPressAgain_cnt;
/***********************************
* �ֲ�����
***********************************/

/*******************************************************************************
** ��������: EnterStopMode
** ��������: ����͹���ģʽ
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
//Ӳ�����󣬵���PA0��PA4����Ի�
void CfgPA4ASWFI()
{
	//ʱ��ʹ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF,ENABLE);  
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	
	#if 0
	//�ⲿ����GPIOA��ʼ��,PA0  
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
	

	//��EXTI0ָ��PA0  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);  
	//EXTI0�ж�������
	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0�ж���������  
	NVIC_InitTypeDef NVIC_InitStructure;  
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	#endif
	
	//PA4 
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	//��EXTI4ָ��PA4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4);  
	//EXTI4�ж�������
	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line4;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  
	
	//EXTI4�ж���������  
	NVIC_InitTypeDef NVIC_InitStructure;  
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
}


//���PA0(WAKE_UP)�Ƿ񱻰���
BOOL Check_wakeUpKey_pressed(void)
{
	//uint32_t cnt=0;
	while(TRUE)
	{
#if 0
//		//��ȡPA0�ĵ�ƽ
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
//		{
//			//cnt++;
//			delay_ms(10);
//			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
//			{
////				while(TRUE)
////				{
////					delay_ms(10);
////					if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
////					{
////						return TRUE;
////					}
////				}
//				return TRUE;
//			}
//		}
//		else
//		{
//			return FALSE;
//		}
#endif
		//��ȡPA4�ĵ�ƽ
		if(GPIO_ReadInputDataBit(KEY_WAKE_UP_PORT, KEY_WAKE_UP_PIN)==0)
		{
			//cnt++;
			delay_ms(10);
			if(GPIO_ReadInputDataBit(KEY_WAKE_UP_PORT, KEY_WAKE_UP_PIN)==0)
			{
//				while(TRUE)
//				{
//					delay_ms(10);
//					if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
//					{
//						return TRUE;
//					}
//				}
				return TRUE;
			}
		}
		else
		{
			return FALSE;
		}
	}
}

//void EXTI0_1_IRQHandler(void)
//{  
//	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
//	{ 
//		#if 0
////		if(Check_wakeUpKey_pressed())
////		{
////			b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;		//ÿ��һ�Σ�b_Is_PCB_PowerOn��תһ��״̬
////			if(b_Is_PCB_PowerOn==TRUE)
////			{
////				mcu_state=POWER_ON;	
////				key_state=KEY_WAKE_UP;		
////				state=LOAD_PARA;
////				init_PWMState();
////			}
////			else
////			{
////				mcu_state=POWER_OFF;	
////				key_state=KEY_STOP_MODE;
////				state=LOAD_PARA;
////				init_PWMState();
////			}
////		}
//		#endif
//		key_state=KEY_DOWNING;
//		wakeup_Cnt=0;
//	}  
//	//EXTI_ClearITPendingBit(EXTI_Line0);
//	EXTI_ClearFlag(EXTI_Line0);
//} 

void EXTI4_15_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)  
	{ 
		key_state=KEY_DOWNING;
		wakeup_Cnt=0;
	}  
	//EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearFlag(EXTI_Line4);
} 


void init_system_afterWakeUp()
{
	Init_iWtdg(4,1250);  //4*2^4=64��Ƶ��1250(�����1250*1.6ms=2s)
	os_ticks = 0;
	//os_ticks = 4294967290;
	
	delay_init();
	os_init();
	
	SystemInit();
	
	init_task();
}


void CfgALLPins4StopMode()
{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, DISABLE); //�ر�RTC��ʱ��ʹ�ܣ���仰Ҫ���ǲ�Ҫ��
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOF, ENABLE);
	
//	//PF0,PF1
	GPIO_InitTypeDef GPIO_InitStructure_PF;
	GPIO_InitStructure_PF.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                       
	GPIO_InitStructure_PF.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure_PF.GPIO_Mode = GPIO_Mode_IN;	
//	GPIO_InitStructure_PF.GPIO_Mode = GPIO_Mode_AF;	
//	GPIO_InitStructure_PF.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PF.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure_PF);
//	GPIO_SetBits(GPIOF, GPIO_Pin_0|GPIO_Pin_1);
	
	//PA0�������ã��Ѿ����ó����ж�
	
	//����ADC1   PA1
	GPIO_InitTypeDef GPIO_InitStructure_PA_1;
	GPIO_InitStructure_PA_1.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure_PA_1.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//	GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure_PA_1.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1);	
	
	//�ر�ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , DISABLE);		
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);

	//PA2,PA3
	GPIO_InitTypeDef GPIO_InitStructure_PA_2_3;
	GPIO_InitStructure_PA_2_3.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                       
	GPIO_InitStructure_PA_2_3.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_2_3.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PA_2_3.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_2_3);
	
	//�رմ���
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
	//PA4����ж��ˣ������������ˣ����ｫPA4�ĳ�PA0        //ԭ��Ӳ���ϵĴ�����PA0��PA4����Ի�
//	GPIO_InitTypeDef GPIO_InitStructure_PA4;
//	GPIO_InitStructure_PA4.GPIO_Pin = GPIO_Pin_4;                       
//	GPIO_InitStructure_PA4.GPIO_Speed = GPIO_Speed_50MHz;   
////	GPIO_InitStructure_PA4.GPIO_Mode = GPIO_Mode_OUT;
////	GPIO_InitStructure_PA4.GPIO_OType=GPIO_OType_PP;
////	GPIO_InitStructure_PA4.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PA4.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA4.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA4);
	GPIO_InitTypeDef GPIO_InitStructure_PA0;
	GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0;                       
	GPIO_InitStructure_PA0.GPIO_Speed = GPIO_Speed_50MHz;   
	GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PA0.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA0);
	
	//PA5
	GPIO_InitTypeDef GPIO_InitStructure_PA5;
	GPIO_InitStructure_PA5.GPIO_Pin = GPIO_Pin_5;                       
	GPIO_InitStructure_PA5.GPIO_Speed = GPIO_Speed_50MHz;   
	GPIO_InitStructure_PA5.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure_PA5.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_InitStructure_PA5.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA5.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA5.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA5);
//	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	
	//PWM1(PA6)
	GPIO_InitTypeDef GPIO_InitStructure_PWM_1;
	GPIO_InitStructure_PWM_1.GPIO_Pin = GPIO_Pin_6;                       
	GPIO_InitStructure_PWM_1.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure_PWM_1.GPIO_Mode = GPIO_Mode_IN;	
//	GPIO_InitStructure_PWM_1.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PWM_1.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PWM_1.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PWM_1.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PWM_1);
	//GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
	
		//PWM2(PB1)
	GPIO_InitTypeDef GPIO_InitStructure_PWM2;
	GPIO_InitStructure_PWM2.GPIO_Pin = GPIO_Pin_1;                       
	GPIO_InitStructure_PWM2.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PWM2.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PWM2.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PWM2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PWM2.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PWM2);
	//GPIO_SetBits(GPIOB,GPIO_Pin_1);
	
		//PA9,PA10
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                       
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	

	
	//PB8,PB9,PB10 LED
	GPIO_InitTypeDef GPIO_InitStructure_LED;
	GPIO_InitStructure_LED.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;                       
	GPIO_InitStructure_LED.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_LED.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_LED.GPIO_PuPd=GPIO_PuPd_UP;
	//GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure_LED);
	GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	
	//PB11
	GPIO_InitTypeDef GPIO_InitStructure_PB11;
	GPIO_InitStructure_PB11.GPIO_Pin = GPIO_Pin_11;                       
	GPIO_InitStructure_PB11.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_PB11.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB11.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB11.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB11);
//	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	
	//PA7,PA8,PA11,PA12,PA15
	GPIO_InitTypeDef GPIO_InitStructure_PA_7_8_11_12_15;
	GPIO_InitStructure_PA_7_8_11_12_15.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;                       
	GPIO_InitStructure_PA_7_8_11_12_15.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_PA_7_8_11_12_15.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PA_7_8_11_12_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_7_8_11_12_15);
	
	//PC13
	GPIO_InitTypeDef GPIO_InitStructure_PC_13;
	GPIO_InitStructure_PC_13.GPIO_Pin = GPIO_Pin_13;                       
	GPIO_InitStructure_PC_13.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_PC_13.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PC_13.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PC_13.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure_PC_13.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure_PC_13);
	
	//PB0,PB2,PB3,PB4,PB5,PB6,PB7,PB12,PB13,PB14,PB15
	GPIO_InitTypeDef GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15;
	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;                       
	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_0_2_3_4_5_6_7_12_13_14_15);
}

void init_global_variant()
{
	checkPressAgain_cnt=0;
	
	b_check_BAT_ok=FALSE;
	switch_mode_cnt=0;
	b_check_bnt_release=FALSE;
	release_btn_cnt=0;
	
	 b_check_BAT_ok=FALSE;
	//honeywell��ز���
	honeywell_state=HONEYWELL_START;
	HONEYWELL_ZERO_POINT=0;
	adc_pressure_value=0;
	b_getHoneywellZeroPoint=0;
	
	b_Is_PCB_PowerOn=FALSE;
	wakeup_Cnt=0;
	key_state=KEY_UPING;
//	b_check_bat=FALSE;
	led_state=LED_NONE;
	led_high_cnt=0;
	led_low_cnt=0;
	flash_cnt=0;
//	thermal_state=THERMAL_NONE;
	//adc_state=1;
	mcu_state=POWER_OFF;
	state=LOAD_PARA;
}

//����stopģʽ�������жϻ���
void EnterStopMode()
{
	init_global_variant();
	init_PWMState();
	//�����ж�
	CfgPA4ASWFI();
	//��оƬ�ˣ�����Ҫ�������
//	//I2CоƬADS115����power-downģʽ
//	ADS115_enter_power_down_mode();

	CfgALLPins4StopMode();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}


LED_STATE Check_Bat()
{
	uint16_t result;
	result=RegularConvData_Tab[0];
	if(result<BATTERY_NO_POWER_THRESHOLD) //�����ѹС��2.2v,û���� ��ֱ�ӽ���͹���  3003
	{ 
//		record_dateTime(CODE_NO_POWER);
		return LED_RED_SOLID_NO_POWER;
	}
	else if(result>=BATTERY_NO_POWER_THRESHOLD&&result<BATTERY_LOW_POWER_THRESHOLD)  //2.2-2.6 �������û�����������
	{
		//led_state=LED_RED_FLASH;
		return LED_RED_FLASH_LOW_POWER;
	}
	else if(result>=BATTERY_LOW_POWER_THRESHOLD)
	{
		//solid green,�����̵ƣ���ʾ������ֵ
		//led_state=LED_GREEN_SOLID;
		return LED_GREEN_SOLID;
	}
	else
	{
		//do nothing
		return LED_NONE;
	}
}


void key_led_task(void)
{
	if(key_state==KEY_STOP_MODE)
	{
		//��¼�ػ�ʱ��
		record_dateTime(CODE_MANUAL_POWER_OFF);
		
		EnterStopMode();
		init_system_afterWakeUp();
	}
	
	if(key_state==KEY_DOWNING)
	{
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
		if(GPIO_ReadInputDataBit(KEY_WAKE_UP_PORT, KEY_WAKE_UP_PIN)==0)
		{
			if(wakeup_Cnt==2)
			{
				wakeup_Cnt=0;
//				b_KeyWkUP_InterrupHappened=FALSE;  //����жϷ�����־
				b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;
				
				if(b_Is_PCB_PowerOn)
				{
					mcu_state=POWER_ON;	
					key_state=KEY_WAKE_UP;		
					state=LOAD_PARA;
					init_PWMState();
				}
				else
				{
					mcu_state=POWER_OFF;	
					key_state=KEY_STOP_MODE;
					state=LOAD_PARA;
					init_PWMState();
				}
			}
			else
			{
				wakeup_Cnt++;
			}
		}
		else
		{
			wakeup_Cnt=0;
			if(!b_Is_PCB_PowerOn)  //b_Is_PCB_PowerOnΪFALSE�ǲŽ����жϣ�����ʱ����̣�����������
			{
//				NVIC_SystemReset();
//				key_state=KEY_FAIL_WAKEUP;
				EnterStopMode();
				init_system_afterWakeUp();
			}
		}
	}
	
	//���������£�����ص�ѹ
	if(key_state==KEY_WAKE_UP)
	{
		led_state=Check_Bat();
		key_state=KEY_UPING;
		
#if 0
//		//b_check_bat=TRUE;
//		//if(b_Is_PCB_PowerOn)
//		{
//			if(RegularConvData_Tab[0]>3003)
//			{
//				if(RegularConvData_Tab[0]>3549)
//				{
//					//TODO  �����Ҫһ����־λ��ʾ���Կ���
////					set_led(LED_GREEN);
//					b_check_BAT_ok=TRUE;
//				}
////				
////				show_mode_LED();
//				
//				led_state=Check_Bat();
//				
//				//��¼����ʱ��
//				record_dateTime(CODE_SYSTEM_POWER_ON);
//				
//				Motor_PWM_Freq_Dudy_Set(1,100,80);
//				Motor_PWM_Freq_Dudy_Set(2,100,80);
//				//Motor_PWM_Freq_Dudy_Set(3,100,80);
//				Delay_ms(500);
//				Motor_PWM_Freq_Dudy_Set(1,100,0);
//				Motor_PWM_Freq_Dudy_Set(2,100,0);
//				//Motor_PWM_Freq_Dudy_Set(3,100,0);
//				
//				key_state=KEY_UPING;
//				mcu_state=POWER_ON;
//			}
//			else
//			{
//				set_led(LED_CLOSE,TRUE);
//				set_led(LED_YELLOW,TRUE);
//				for(int i=0;i<5;i++)
//				{
//					Motor_PWM_Freq_Dudy_Set(1,100,0);
//					Motor_PWM_Freq_Dudy_Set(2,100,0);
////					Motor_PWM_Freq_Dudy_Set(3,100,0);
//					Delay_ms(500);
//					Motor_PWM_Freq_Dudy_Set(1,100,50);
//					Motor_PWM_Freq_Dudy_Set(2,100,50);
////					Motor_PWM_Freq_Dudy_Set(3,100,50);
//					Delay_ms(500);
//					IWDG_Feed();
//				}
//				mcu_state=POWER_OFF;
//				
//				//����stopģʽ
//				EnterStopMode();
//				//����֮�����³�ʼ��
//				init_system_afterWakeUp();
//			}
//		}
#endif
	}

	//IWDG_Feed();   //ι��
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
