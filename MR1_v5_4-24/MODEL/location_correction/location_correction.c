#include "main.h"


// 利用色标传感器进行位置矫正
void lc_color_sensor_location_correction(void)
{
	// 判断处于哪一段路径
	// 列举了可能会用到色标矫正功能的路径
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

// 启动区到交接区利用色标进行位置矫正
void lc_color_MOVING_URTUU_TO_RZ(void)
{
	// 判断位于哪一段路径
	switch(PATH_POINT_COUNTER)
	{
		// 第一段绕柱弧线
		case 1:
		{
			// 更新X轴
			if(cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_INSIDE_LINE_X + DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}
			
			// 更新Y轴
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_1_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}
		}
		break;
		
		// 第二段绕柱弧线
		case 2:
		{
			// 更新X轴
			if(cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_OUTSIDE_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}
			
			// 更新Y轴
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_2_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// 第三段绕柱弧线（之前圆弧的一半）
		case 3:
		{
			// 更新X轴			
			if(cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == RESET)
			{
				GL_POS_DATA.X = VERTICAL_INSIDE_LINE_X + DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// 更新Y轴
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_3_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// 柱后第一段相切线
		case 4:
		{
			// 更新X轴(需考虑红蓝场差异）			
			if((cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET) &&
				 (cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == SET))
			{
				GL_POS_DATA.X = VERTICAL_CENTER_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// 更新Y轴
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_4_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;
		
		// 柱后第二段相切线
		case 5:
		{
			// 更新X轴(需考虑红蓝场差异）			
			if((cs_read_color_sensor_status(VERTICAL_RIGHT_COLOR_SENSOR) == RESET) &&
				 (cs_read_color_sensor_status(VERTICAL_LEFT_COLOR_SENSOR) == SET))
			{
				GL_POS_DATA.X = VERTICAL_CENTER_LINE_X - DISTANCE_CENTER_TO_VERTICAL_SENSOR;
			}	

			// 更新Y轴
			if((cs_read_color_sensor_status(ACLINIC_LEFT_COLOR_SENSOR)  == RESET) && 
				 (cs_read_color_sensor_status(ACLINIC_RIGHT_COLOR_SENSOR) == RESET))
			{
				GL_POS_DATA.Y = ACLINIC_LINE_4_Y + DISTANCE_CENTER_TO_ACLINIC_SENSOR;
			}			
		}
		break;

		// 过桥的直线
		case 6:
		{
			// 更新Y轴
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


// 利用激光测距进行位置矫正
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
