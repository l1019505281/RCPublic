#include "main.h"


ENCO_DIS ENCO_DIS_DATA = {0, 0, 0, 0};
GL_POS GL_POS_DATA = {0, 0, 0};


// ��ʼ��ȫ����λģ��
void gl_GL_model_init(void)
{
	u8 time_counter = 0;
	
	// ��ʼ���ײ�����
	encoder_TIM4_init();
  encoder_TIM8_init();
	gyro_UART4_init(115200);
	
	// �ȴ������������뷢������ ��ȡ������Z���ʼ�Ƕ�
	// ����ѭ����ʼǰ��ʱ����Ӱ��ʱ��
	while(time_counter < 100)
	{
		time_counter ++;
		delay_ms(5);
	}
	
	// ����λ��ƫ���� �����������������λ��Ϊԭ�㣩*************************��ĵ�FSM��ѡ��ģʽ������
	ROBOT_POS_OFFSET_DATA.X_CS_OFFSET = -500.0;
	ROBOT_POS_OFFSET_DATA.Y_CS_OFFSET =  500.0;
	ROBOT_POS_OFFSET_DATA.YAW_CS_OFFSET = 0.0;
	
	GL_POS_DATA.X = ROBOT_POS_OFFSET_DATA.X_CS_OFFSET;
	GL_POS_DATA.Y = ROBOT_POS_OFFSET_DATA.Y_CS_OFFSET;
	
	// ��ȡ��ʼƫ����
	ROBOT_POS_OFFSET_DATA.YAW_IN_OFFSET = -(float)GYRO_ANGLE.Angle[2] / 32768 * 180;  
}


// ����ȫ����λ��λλ������
void gl_update_GL_position(void)
{
	// ��ȡ��ʱ��CNT�Ĵ��������ֵ
	// �����˶���ֵӦ��Ϊ���������������������
	ENCO_DIS_DATA.CNT_X =  -encoder_get_encoder_CNT(4);  // ��˳ʱ��������
	ENCO_DIS_DATA.CNT_Y = -encoder_get_encoder_CNT(8);   // ��˳ʱ��β����

	// ����������н���·��ֵ
	ENCO_DIS_DATA.DELTA_DIS_X = ENCO_DIS_DATA.CNT_X * DISTANCE_PER_CNT;
	ENCO_DIS_DATA.DELTA_DIS_Y = ENCO_DIS_DATA.CNT_Y * DISTANCE_PER_CNT;
	
	// ���������н���·��ֵת��ȫ����λ��������λ��
	gl_encoder_to_GL_conversion();
}


// ���������н���·��ֵת��ȫ����λ��������λ��
void gl_encoder_to_GL_conversion(void)
{
	static float last_yaw = 0;  // ��һ�ε�ƫ����
	float delta_rad = 0;  // ƫ���ǻ��Ȳ�
	float model_yaw = 0;  // ģ������ϵƫ����
		
	// ֱ�ӻ�ȡƫ���ǣ������м򵥴���ƫ���Ƕ�ֵȫ����ʱ��Ϊ����˳ʱ��Ϊ����
	GL_POS_DATA.YAW = (float)GYRO_ANGLE.Angle[2] / 32768 * 180 - ROBOT_POS_OFFSET_DATA.YAW_IN_OFFSET + ROBOT_POS_OFFSET_DATA.YAW_CS_OFFSET;  /*WAITING_TEST*/
	// ���Ʒ�Χ(-180�� 180]
	if(GL_POS_DATA.YAW > 180) {GL_POS_DATA.YAW = -180 + (GL_POS_DATA.YAW - 180);}
	else if(GL_POS_DATA.YAW <= -180) {GL_POS_DATA.YAW = 180 + (GL_POS_DATA.YAW + 180);}
	
	// ����ģ������ϵƫ����
	model_yaw = GL_POS_DATA.YAW + YAW0;
	// ���Ʒ�Χ(-180�� 180]	
	if(model_yaw > 180) {model_yaw = -180 + (model_yaw - 180);}
	else if(model_yaw <= -180) {model_yaw = 180 + (model_yaw + 180);}
	
	// ����ƫ���ǻ��Ȳ���浱ǰƫ����
	if(last_yaw*GL_POS_DATA.YAW >= 0)
	{
		delta_rad = (GL_POS_DATA.YAW - last_yaw) / 180.0f * PI;
	}
	else
	{
		if(float_abs(last_yaw) + float_abs(GL_POS_DATA.YAW) <= 180) delta_rad = (GL_POS_DATA.YAW - last_yaw) / 180.0f * PI;
		else
		{
			if((GL_POS_DATA.YAW - last_yaw) > 0)
			{
				delta_rad = (GL_POS_DATA.YAW - last_yaw - 360) / 180.0f * PI;
			}
			else if((GL_POS_DATA.YAW - last_yaw) < 0)
			{
				delta_rad = (GL_POS_DATA.YAW - last_yaw + 360) / 180.0f * PI;
			}
		}
	}
	last_yaw = GL_POS_DATA.YAW;
	
	// �˶�ѧ����
	GL_POS_DATA.X += (ENCO_DIS_DATA.DELTA_DIS_X - Lx * delta_rad) * cos(model_yaw / 180.0f * PI) - 
									 (ENCO_DIS_DATA.DELTA_DIS_Y + Ly * delta_rad) * sin(model_yaw / 180.0f * PI);
	
	GL_POS_DATA.Y += (ENCO_DIS_DATA.DELTA_DIS_X - Lx * delta_rad) * sin(model_yaw / 180.0f * PI) + 
	                 (ENCO_DIS_DATA.DELTA_DIS_Y + Ly * delta_rad) * cos(model_yaw / 180.0f * PI);

}








