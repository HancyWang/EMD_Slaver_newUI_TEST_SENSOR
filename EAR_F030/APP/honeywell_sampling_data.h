#ifndef _HONEYWELL_SAMPLING_DATA
#define _HONEYWELL_SAMPLING_DATA

#include "datatype.h"
#include "stdint.h"
#define DEBUG_STORE_HONEYWELL_DATA
//#define HONEYWELL_STRC_DATA_SIZE 30
#define HONEYWELL_STRC_DATA_SIZE 15   //每次传输的数据个数，3个数据，每个数据2个字节

#define HONEYWELL_RATE			11185   //斜率,根据公式算出来的
//#define PRESSURE_SAFETY_THRESHOLD 20   //20mmHg，最大过压值
#define PRESSURE_SAFETY_THRESHOLD 100   	//应Ilan要求，改成100做测试
#define PRESSURE_SENSOR_VALUE(x) (((HONEYWELL_RATE)*(x))+(HONEYWELL_ZERO_POINT))


#define HONEYWELL_SAMPLING_DATA_PERIOD 5


/// @brief SENSOR_DATA struct 
/// @brief Date:
/// - YEAR 
/// - MONTH 
/// - DAY 
/// - HOUR 
/// - MIN 
/// - SECOND 
/// @brief data:
/// - DATA_0 - integer part
/// - DATA_1 - fractional part

//typedef struct
//{
//	uint8_t YEAR;
//	uint8_t MONTH;
//	uint8_t DAY;
//	uint8_t HOUR;
//	uint8_t MIN;
//	uint8_t SECOND;
//	uint8_t DATA_0;
//	uint8_t DATA_1;
//}SENSOR_DATA;


typedef enum 
{
	HONEYWELL_START,
	HONEYWELL_NONE,
//	HONEYWELL_WAIT_5ms,
	HONEYWELL_READ_DATA,
	HONEYWELL_SAMPLE_DATA_FINISH
}HONEYWELL_STATE;

extern BOOL b_Start_Store_Data;

void honeywell_sampling_data(void);
UINT32 trans_xmmHg_2_adc_value(UINT8 xmmHg);
#endif //_HONEYWELL_SAMPLING_DATA


