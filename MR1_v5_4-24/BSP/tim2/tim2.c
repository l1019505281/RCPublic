#include "main.h"

TIM2_TASK_COUNTER TIM2_TASK_COUNTER_DATA = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// �������������ʱ��TIM2
void tim2_TIM2_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // ��ʱ��
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 1000 * 1000 / 1000 - 1;  // 1ms����һ������ж�
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);  // ����жϱ�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  // ��������ж�
}


// ��ʼ������ѭ��
void tim2_start_main_loop(void)
{
	TIM_Cmd(TIM2, ENABLE);	 // ʹ�ܼ�ʱ��
}


// TIM2����жϺ���
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) //����ж�
	{
		TIM2_TASK_COUNTER_DATA.COUNTER_IT     ++;  // ��¼�����ж϶��ٴΣ�ÿ1ms����һ�Σ�
		TIM2_TASK_COUNTER_DATA.COUNTER_1MS    ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_2MS    ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_5MS    ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_10MS   ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_20MS   ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_50MS   ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_100MS  ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_200MS  ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_500MS  ++;
		TIM2_TASK_COUNTER_DATA.COUNTER_1000MS ++;
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //����жϱ�־λ 
	} 
}




