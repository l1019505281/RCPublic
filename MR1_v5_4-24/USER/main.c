#include "main.h"

int main(void)
{	
	extern struct PLANNING_PATH DEBUG_PLANNING_PATH[];

	NOW_PATH = DEBUG_PLANNING_PATH;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	uart_init(115200);
	
	bsp_pre_charge();
	gl_GL_model_init();
	can1_CAN1_init();
	cd_chassis_driver_init();
	
	adc_adc3_init();
	moving_init();
	tim2_TIM2_init();
	tim2_start_main_loop();
	
//	queue *ADC3_value;
//	if(OK == initQueue(ADC3_value,100) )printf("Queue Init success");
//	else printf("Queue Init error");
	
	task_frequency_control_main_loop();
}



