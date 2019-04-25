#include "main.h"


// ��ǿ������Ԥ��粢��ʼ�����GPIO
// PB15 -> charge_pin
void bsp_pre_charge(void)
{
  // ��ʼ��GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  // ��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // �������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Ԥ���
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	
	// ��ʱ
	delay_ms(PRE_CHARGE_TIME);
	
	// ��ʼʹ��
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

