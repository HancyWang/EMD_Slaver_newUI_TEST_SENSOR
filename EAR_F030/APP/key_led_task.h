#ifndef __KEY_TASK_H_
#define __KEY_TASK_H_
/***********************************
* ͷ�ļ�
***********************************/

/**********************************
*�궨��
***********************************/
#define KEY_LED_PERIOD  20
#define CHECK_MODE_OUTPUT_PWM 10
#define ONE_SEC_KEY_TIME   1000/KEY_LED_PERIOD
/***********************************
* ȫ�ֱ���
***********************************/

#define KEY_WAKE_UP_PORT 	GPIOA
#define KEY_WAKE_UP_PIN		GPIO_Pin_4

#define KEY_MODE_PORT			GPIOA
#define KEY_MODE_PIN			GPIO_Pin_0

typedef enum
{
	POWER_ON,
	POWER_OFF
}MCU_STATE;

typedef enum
{
	KEY_STOP_MODE,
	KEY_UPING,
	KEY_DOWNING,
//	KEY_DOWN_UP,
	KEY_WAKE_UP
	//KEY_UP_DOWN
}KEY_STATE;

typedef enum
{
	LED_NONE,
	LED_RED_SOLID_NO_POWER,
	LED_RED_FLASH_LOW_POWER,
	LED_GREEN_SOLID
}LED_STATE;

/***********************************
* ��Ͷ��x
***********************************/

/***********************************
* �ⲿ����
***********************************/
void key_led_task(void);
void EnterStopMode(void);
void CfgPA4ASWFI(void);
void CfgALLPins4StopMode(void);
void init_system_afterWakeUp(void);
//extern INT8U I2C_RecByte(void);
LED_STATE Check_Bat(void);


#endif
