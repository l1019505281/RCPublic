#include "main.h"


// ����ɫ�괫��������λ�ý���
void lc_color_sensor_location_correction(void)
{
	// �жϴ�����һ��·��
	// �о��˿��ܻ��õ�ɫ��������ܵ�·��
	switch(NOW_STATE)
	{
		case MOVING_URTUU_TO_RZ:
		{
			lc_color_MOVING_URTUU_TO_RZ();
		}
		break;
		
		case MOVING_PZ_TO_TZ:
		{

		}
		break;
		
		case MOVING_S1_TO_TZ:
		{

		}
		break;
		
		case MOVING_S2_TO_TZ:
		{

		}
		break;
		
		case MOVING_S3_TO_TZ:
		{

		}
		break;
		
		case MOVING_TZ_TO_S1:
		{

		}
		break;
		
		case MOVING_TZ_TO_S2:
		{

		}
		break;

		case MOVING_TZ_TO_S3:
		{

		}
		break;
		
		default: break;
	}
}

// ������������������ɫ�����λ�ý���
void lc_color_MOVING_URTUU_TO_RZ(void)
{
	// �ж�λ����һ��·��
	switch(PATH_POINT_COUNTER)
	{
		// ��һ����������
		case 1:
		{
			// ����X��
			if(cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_INSIDE_LINE_X + DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}
			
			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_1_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}
		}
		break;
		
		// �ڶ�����������
		case 2:
		{
			// ����X��
			if(cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_OUTSIDE_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}
			
			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_2_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// �������������ߣ�֮ǰԲ����һ�룩
		case 3:
		{
			// ����X��			
			if(cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_INSIDE_LINE_X + DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_3_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// �����һ��������
		case 4:
		{
			// ����X��(�迼�Ǻ��������죩			
			if((cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET) &&
				 (cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == SET))
			{
				GL_POS_DATA.X = VERTICAL_CENTER_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_4_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;
		
		// ����ڶ���������
		case 5:
		{
			// ����X��(�迼�Ǻ��������죩			
			if((cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET) &&
				 (cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == SET))
			{
				GL_POS_DATA.X = VERTICAL_CENTER_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_4_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// ���ŵ�ֱ��
		case 6:
		{
			// ����Y��
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_BRIDGE_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}				
		}
		break;
		
		default: break;
	}
}


/*************************************************************************************/


// ���ü��������λ�ý���
void lc_color_laser_ranging_location_correction(void)
{
	
}

//	switch(NOW_STATE)
//	{
//		case MOVING_URTUU_TO_RZ:
//		{

//		}
//		break;
//		
//		case MOVING_LINE1_TO_RZ:
//		{

//		}
//		break;
//		
//		case MOVING_RZ_TO_S1:
//		{

//		}
//		break;
//		
//		case MOVING_LINE1_TO_S1:
//		{

//		}
//		break;
//		
//		case MOVING_LINE1_TO_S2:
//		{

//		}
//		break;
//		
//		case MOVING_LINE1_TO_S3:
//		{

//		}
//		break;
//		
//		case MOVING_S1_TO_PZ:
//		{

//		}
//		break;
//		
//		case MOVING_PZ_TO_TZ:
//		{

//		}
//		break;
//		
//		case MOVING_S1_TO_TZ:
//		{

//		}
//		break;
//		
//		case MOVING_S2_TO_TZ:
//		{

//		}
//		break;
//		
//		case MOVING_S3_TO_TZ:
//		{

//		}
//		break;
//		
//		case MOVING_TZ_TO_S1:
//		{

//		}
//		break;
//		
//		case MOVING_TZ_TO_S2:
//		{

//		}
//		break;

//		case MOVING_TZ_TO_S3:
//		{

//		}
//		break;
//	}
