#include "main.h"


// 色标传感器所用的GPIO初始化
// 竖直左侧色标 -> PE7
// 水平左侧色标 -> PE8
// 水平右侧色标 -> PE9
// 竖直右侧色标 -> PE10
// NPN型 触发接地
void cs_GPIO_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;		
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  // 开时钟

	//初始化相应的GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  // 普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // 上拉
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}


// 获取色标传感器电平状态
// 低电平返回0(RESET)
// 高电平返回1(SET)
uint8_t cs_read_color_sensor_status(u8 sensor)
{
	uint8_t status = SET;  // 默认上拉
	
	switch(sensor)
	{
		case VERTICAL_LEFT_COLOR_SENSOR : status = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) ; break;
		case ACLINIC_LEFT_COLOR_SENSOR  : status = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) ; break;		
		case ACLINIC_RIGHT_COLOR_SENSOR : status = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) ; break;
		case VERTICAL_RIGHT_COLOR_SENSOR: status = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10); break;
		default: break;
	}
	
	return status;
}

