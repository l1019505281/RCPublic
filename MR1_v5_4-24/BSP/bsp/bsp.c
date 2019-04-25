#include "main.h"


// 对强电板进行预充电并初始化相关GPIO
// PB15 -> charge_pin
void bsp_pre_charge(void)
{
  // 初始化GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  // 普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// 预充电
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	
	// 延时
	delay_ms(PRE_CHARGE_TIME);
	
	// 开始使用
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

