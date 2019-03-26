#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"

#include "delay.h"
#include "os_cfg.h"
#include "hardware.h"
#include "app.h"

#include "comm_task.h"
#include "key_led_task.h"
#include "hardware.h"
#include "i2c.h"
#include "Motor_pwm.h"
//#include "device_type.h"
#include "stm32f0xx_dma.h"
#include "iwtdg.h"
#include "app.h"

 const uint8_t default_parameter_buf[PARAMETER_BUF_LEN] = {
#if 1
16,//1.0,1*16+0=16
1,

//MODE1
0x11,1,100,40,30,1,0,0,
0x12,1,100,30,25,1,0,1,
0x13,1,100,40,30,1,0,0,
0x14,1,100,30,25,1,0,1,
0x15,1,100,40,30,1,0,0,
0x16,1,100,30,25,1,0,0,

0x21,1,100,25,1,1,0,14,
0x22,1,100,25,16,1,0,39,
0x23,1,100,25,16,1,0,39,
0x24,1,100,25,16,1,0,39,
0x25,0,100,25,1,1,0,14,
0x26,0,100,25,15,1,0,1,

0x31,0,100,30,1,1,1,1,
0x32,0,100,40,1,2,1,2,
0x33,0,100,50,1,3,1,3,
0x34,0,100,60,1,4,1,4,
0x35,0,100,70,1,5,1,5,
0x36,0,100,80,1,6,1,6,


//MODE2
0x11,1,100,55,15,1,0,15,
0x12,1,100,55,15,1,0,16,
0x13,1,100,55,15,1,0,15,
0x14,1,100,55,15,1,0,16,
0x15,1,100,55,15,2,0,15,
0x16,1,100,30,14,1,0,0,

0x21,1,100,40,1,1,0,14,
0x22,1,100,40,31,1,0,0,
0x23,1,100,30,14,1,0,16,
0x24,1,100,40,31,1,0,0,
0x25,1,100,30,14,1,0,16,
0x26,1,100,40,31,1,0,0,

0x31,0,100,30,1,1,1,1,
0x32,0,100,40,1,2,1,2,
0x33,0,100,50,1,3,1,3,
0x34,0,100,60,1,4,1,4,
0x35,0,100,70,1,5,1,5,
0x36,0,100,80,1,6,1,6,


//MODE3
0x11,1,100,99,15,1,0,15,
0x12,1,100,99,25,1,0,16,
0x13,1,100,99,15,1,0,15,
0x14,1,100,99,25,1,0,16,
0x15,1,100,99,15,1,0,15,
0x16,1,100,99,25,1,0,0,

0x21,1,100,66,1,1,0,14,
0x22,1,100,66,41,1,0,0,
0x23,1,100,30,14,1,0,16,
0x24,1,100,66,41,1,0,0,
0x25,1,100,30,14,1,0,16,
0x26,1,100,66,55,1,0,0,

0x31,0,100,30,1,1,1,1,
0x32,0,100,40,1,2,1,2,
0x33,0,100,50,1,3,1,3,
0x34,0,100,60,1,4,1,4,
0x35,0,100,70,1,5,1,5,
0x36,0,100,80,1,6,1,6,


//Checksum
0x2C,0xD9
		#endif
	};

//The new UI version start from:1.1.0
//And the old UI version start from: 1.0.0
const uint8_t SW_VERSION[3]={1,1,0};
	
int main(void)
{
	
  delay_init();
	os_init();
	
	#ifdef _DEBUG
	#else
	//����stopģʽ
	EnterStopMode();
	//����֮���ȳ�ʼ��ϵͳ
	init_system_afterWakeUp();
	#endif

	os_create_task(init_task, OS_TRUE, INIT_TASK_ID);
	os_start();

	return 0;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
