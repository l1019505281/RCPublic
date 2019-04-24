#include "main.h"
#include "path.h"


PLANNING_PATH *NOW_PATH = NULL;  // ��ǰ·����ָ��(��ȷ��ָ��ĳ��·���ٽ���·�����ٺ�����

// �����õı���
robot_struct mr1 = {0.0, 0.0, 0.0};
arc_struct arc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
line_struct line = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
point_struct point = {0.0, 0.0, 0.0};
trapezoidal_V_struct traV = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
yaw_struct yaw = {0.0, 0.0, 0.0};

u32 PATH_POINT_COUNTER = 0;     // ���ڱ���·���ϵ���ɢ��ı���
u32 PATH_POINT_SUM = 0;         // ��ǰ·����ɢ�������
u32 YAW_TIME_COUNTER = 0;       // ƫ����ʱ��Լ��������ʱ��

u8 (*switch_determine)() = NULL;  // ָ���л���ʽ�ĺ���ָ��

PID arc_pid;
PID line_pid;
PID point_x_pid;
PID point_y_pid;
PID yaw_pid;

// ����һ�������ڸ����͵ľ���ֵ����
float float_abs(float value)
{
	if(value < 0)
	{
		value = -value;
	}
	return value;
}


// ʵ�ֻ����˵��ƶ�����
void moving_action_task(void)
{
	// �����˴����ƶ�״̬
	if(MOVING_STATE == MOVING_GOING)
	{
		// ѡ������˵�ǰ���ƶ�·��
		switch(NOW_STATE)
		{
			case MOVING_URTUU_TO_RZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(URTUU_TO_RZ_PATH)/sizeof(URTUU_TO_RZ_PATH[0]));  // ����ö�·��С·��������
				NOW_PATH = URTUU_TO_RZ_PATH;                                                   // ·��ָ��ָ���Ӧ·��
				moving_select_switch_mode_URTUU_TO_RZ();                                       // ѡ��ͬ���л�·����ʽ
			}
			break;
			case MOVING_LINE1_TO_RZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(LINE1_TO_RZ_PATH)/sizeof(LINE1_TO_RZ_PATH[0])); 
				NOW_PATH = LINE1_TO_RZ_PATH;
			}
			break;
			case MOVING_RZ_TO_S1:
			{
				PATH_POINT_SUM = (u32)(sizeof(RZ_TO_S1_PATH)/sizeof(RZ_TO_S1_PATH[0])); 
				NOW_PATH = RZ_TO_S1_PATH;
			}
			break;
			case MOVING_LINE1_TO_S1:
			{
				PATH_POINT_SUM = (u32)(sizeof(LINE1_TO_S1_PATH)/sizeof(LINE1_TO_S1_PATH[0])); 
				NOW_PATH = LINE1_TO_S1_PATH;
			}
			break;
			case MOVING_LINE1_TO_S2:
			{
				PATH_POINT_SUM = (u32)(sizeof(LINE1_TO_S2_PATH)/sizeof(LINE1_TO_S2_PATH[0])); 
				NOW_PATH = LINE1_TO_S2_PATH;
			}
			break;
			case MOVING_LINE1_TO_S3:
			{
				PATH_POINT_SUM = (u32)(sizeof(LINE1_TO_S3_PATH)/sizeof(LINE1_TO_S3_PATH[0])); 
				NOW_PATH = LINE1_TO_S3_PATH;
			}
			break;
			case MOVING_S1_TO_PZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(S1_TO_PZ_PATH)/sizeof(S1_TO_PZ_PATH[0])); 
				NOW_PATH = S1_TO_PZ_PATH;
			}
			break;
			case MOVING_PZ_TO_TZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(PZ_TO_TZ_PATH)/sizeof(PZ_TO_TZ_PATH[0])); 
				NOW_PATH = PZ_TO_TZ_PATH;
			}
			break;
			case MOVING_S1_TO_TZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(S1_TO_TZ_PATH)/sizeof(S1_TO_TZ_PATH[0])); 
				NOW_PATH = S1_TO_TZ_PATH;
			}
			break;
			case MOVING_S2_TO_TZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(S2_TO_TZ_PATH)/sizeof(S2_TO_TZ_PATH[0])); 
				NOW_PATH = S2_TO_TZ_PATH;
			}
			break;
			case MOVING_S3_TO_TZ:
			{
				PATH_POINT_SUM = (u32)(sizeof(S3_TO_TZ_PATH)/sizeof(S3_TO_TZ_PATH[0])); 
				NOW_PATH = S3_TO_TZ_PATH;
			}
			break;
			case MOVING_TZ_TO_S1:
			{
				PATH_POINT_SUM = (u32)(sizeof(TZ_TO_S1_PATH)/sizeof(TZ_TO_S1_PATH[0])); 
				NOW_PATH = TZ_TO_S1_PATH;
			}
			break;
			case MOVING_TZ_TO_S2:
			{
				PATH_POINT_SUM = (u32)(sizeof(TZ_TO_S2_PATH)/sizeof(TZ_TO_S2_PATH[0])); 
				NOW_PATH = TZ_TO_S2_PATH;
			}
			break;
			case MOVING_TZ_TO_S3:
			{
				PATH_POINT_SUM = (u32)(sizeof(TZ_TO_S3_PATH)/sizeof(TZ_TO_S3_PATH[0])); 
				NOW_PATH = TZ_TO_S3_PATH;
			}
			break;
		}
		
		moving_path_tracking();
	}
	
	//�����˴�����������״̬
	else
	{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
	}
}


// �ƶ����ܳ�ʼ��
void moving_init(void)  //***********************************************************************����
{
	// ��ʼ��PID����
	PID_parameter_init(&arc_pid, 10, 0.04, 2, 0, 0);
	PID_parameter_init(&line_pid, 10, 0.04, 2, 0, 0); 
	PID_parameter_init(&point_x_pid, 10, 0.04, 2, 0, 0);
	PID_parameter_init(&point_y_pid, 10, 0.04, 2, 0, 0);
	PID_parameter_init(&yaw_pid, 3.1, 0.005, 11.25, PI*YAW_OUTPUTMAX_CONTROLLER, PI*YAW_OUTPUTMAX_CONTROLLER);
	
	// �ɼ�����΢������
	
}


/*************************************************************************************/


// ·������
void moving_path_tracking(void)
{
	static u8 counter_zero_flag = 1;
	
	// �ж��Ƿ�Ϊ·����ͷ
	if((PATH_POINT_COUNTER == 0) && (counter_zero_flag == 1))
	{
		moving_assign_path_parameter();  // ���ڵ�ǰ·������
		counter_zero_flag = 0;
	}
	
	// �ж��Ƿ�Ӧ���л�·��,�ı�·������
	if((*switch_determine)() == 1)
	{
		// ��ʱ������
		YAW_TIME_COUNTER = 0;
		
		// ����PID
		PID_reset_PID(&arc_pid);
		PID_reset_PID(&line_pid);
		PID_reset_PID(&yaw_pid);
		PID_reset_PID(&point_x_pid);
		PID_reset_PID(&point_y_pid);

		if(PATH_POINT_COUNTER < PATH_POINT_SUM-1)    // ���������һ����
		{
			PATH_POINT_COUNTER ++;
			moving_assign_path_parameter();  // ���ڵ�ǰ·������
		}
		else  // �������һ����
		{
			PATH_POINT_COUNTER = 0;
			counter_zero_flag = 1;
			MOVING_STATE = MOVING_ARRIVED;  // �л��ƶ�״̬
		}
	}
	
	// ������ʵλ�����ݣ����ڼ���
	mr1.x = ROBOT_REAL_POS_DATA.X_POS;
	mr1.y = ROBOT_REAL_POS_DATA.Y_POS;
	mr1.yaw = ROBOT_REAL_POS_DATA.YAW_POS;
	
	// ѡ��·�����ٷ�ʽ���ٶȹ滮���������棩
	switch(NOW_PATH[PATH_POINT_COUNTER].track_type)
	{
		case 0 : moving_arc_track(); break;
		case 1 : moving_line_track(); break;
		case 2 : moving_point_track(); break;
		default : break;
	}
	
	// ѡ��ƫ���ǵ�����ʽ
	switch(NOW_PATH[PATH_POINT_COUNTER].yaw_adjusting_type)
	{
		case 0 : moving_yaw_adjusting_pure_PID(); break;
		case 1 : moving_yaw_adjusting_time_and_PID(); break;
		default : break;
	}
}


// �����Ӧ�������·������ֵ
void moving_assign_path_parameter(void)
{
	// ƫ����
	yaw.yaw_t1 = NOW_PATH[PATH_POINT_COUNTER].yaw_t1;
	yaw.yaw_t2 = NOW_PATH[PATH_POINT_COUNTER].yaw_t2;
	yaw.yaw = NOW_PATH[PATH_POINT_COUNTER].yaw;

	// ·�����ٶȹ滮��
	switch(NOW_PATH[PATH_POINT_COUNTER].track_type)
	{
		case 0 : 
		{
			arc.x = NOW_PATH[PATH_POINT_COUNTER].x0_or_xcenter;
			arc.y = NOW_PATH[PATH_POINT_COUNTER].y0_or_ycenter;
			arc.R = NOW_PATH[PATH_POINT_COUNTER].x1_or_R;
			arc.Vstart = NOW_PATH[PATH_POINT_COUNTER].Vstart;
			arc.a1 = NOW_PATH[PATH_POINT_COUNTER].a1;
			arc.Vmax = NOW_PATH[PATH_POINT_COUNTER].Vmax;
			arc.a2 = NOW_PATH[PATH_POINT_COUNTER].a2;
			arc.Vend = NOW_PATH[PATH_POINT_COUNTER].Vend;
			arc.sita = NOW_PATH[PATH_POINT_COUNTER].sita;
			arc.d_sita = NOW_PATH[PATH_POINT_COUNTER].d_sita;

			if(NOW_PATH[PATH_POINT_COUNTER].velocity_planning_type == 1)
			{
				traV.s_total = float_abs(arc.d_sita * arc.R);
				traV.s_ac = (arc.Vmax * arc.Vmax - arc.Vstart * arc.Vstart) / (2.0f * arc.a1);
				traV.s_de = (arc.Vend * arc.Vend - arc.Vmax * arc.Vmax) / (2.0f * arc.a2);	
				traV.s_co = traV.s_total - traV.s_ac - traV.s_de;
			} 
			
			// ����ʼ��ָ��Բ����ĵ�λ����
			if(arc.sita == 0)
			{
				traV.sita_vector_x = 1.0f;
				traV.sita_vector_y = 0.0f;
			}
			else if(arc.sita == PI/2.0f)
			{
				traV.sita_vector_x = 0.0f;
				traV.sita_vector_y = 1.0f;
			}
			else if(arc.sita == PI)
			{
				traV.sita_vector_x = -1.0f;
				traV.sita_vector_y = 0.0f;
			}
			else if(arc.sita == -PI/2.0f)
			{
				traV.sita_vector_x = 0.0f;
				traV.sita_vector_y = -1.0f;
			}
			else
			{
				traV.sita_vector_x = cos(arc.sita);
				traV.sita_vector_y = sin(arc.sita);				
			}
			break;
		}
		case 1 : 
		{
			line.x0 = NOW_PATH[PATH_POINT_COUNTER].x0_or_xcenter;
			line.y0 = NOW_PATH[PATH_POINT_COUNTER].y0_or_ycenter;
			line.x1 = NOW_PATH[PATH_POINT_COUNTER].x1_or_R;
			line.y1 = NOW_PATH[PATH_POINT_COUNTER].y1_or_none;
			line.Vstart = NOW_PATH[PATH_POINT_COUNTER].Vstart;
			line.a1 = NOW_PATH[PATH_POINT_COUNTER].a1;
			line.Vmax = NOW_PATH[PATH_POINT_COUNTER].Vmax;
			line.a2 = NOW_PATH[PATH_POINT_COUNTER].a2;
			line.Vend = NOW_PATH[PATH_POINT_COUNTER].Vend;
			
			if(NOW_PATH[PATH_POINT_COUNTER].velocity_planning_type == 1)
			{
				traV.s_total = sqrt((line.y1 - line.y0) * (line.y1 - line.y0) + (line.x1 - line.x0) * (line.x1 - line.x0));
				traV.s_ac = (line.Vmax * line.Vmax - line.Vstart * line.Vstart) / (2.0f * line.a1);
				traV.s_de = (line.Vend * line.Vend - line.Vmax * line.Vmax) / (2.0f * line.a2);	
				traV.s_co = traV.s_total - traV.s_ac - traV.s_de;
			} 
			
			break;
		}
		case 2 :				
		{
			point.x = NOW_PATH[PATH_POINT_COUNTER].x0_or_xcenter;
			point.y = NOW_PATH[PATH_POINT_COUNTER].y0_or_ycenter;
			point.Vmax = NOW_PATH[PATH_POINT_COUNTER].Vstart;
			break;
		}
		default : break;
	}
}


void moving_select_switch_mode_URTUU_TO_RZ(void)
{
	switch(PATH_POINT_COUNTER)
	{
		case 0 : switch_determine = moving_URTUU_TO_RZ_switch_mode_0; break;
		case 1 : switch_determine = moving_URTUU_TO_RZ_switch_mode_1; break;
		case 2 : switch_determine = moving_URTUU_TO_RZ_switch_mode_2; break;
		case 3 : switch_determine = moving_URTUU_TO_RZ_switch_mode_3; break;
		case 4 : switch_determine = moving_URTUU_TO_RZ_switch_mode_4; break;
		case 5 : switch_determine = moving_URTUU_TO_RZ_switch_mode_5; break;
		case 6 : switch_determine = moving_URTUU_TO_RZ_switch_mode_6; break;
		case 7 : switch_determine = moving_URTUU_TO_RZ_switch_mode_7; break;
		case 8 : switch_determine = moving_URTUU_TO_RZ_switch_mode_8; break;	
		default : break;
	}
}


// ��㵽��һ����������
u8 moving_URTUU_TO_RZ_switch_mode_0(void)
{
	if(ROBOT_REAL_POS_DATA.X_POS < -1225) return 1;
//	if(2 == 1) return 1;
	else return 0;
}


// ��һ����������
u8 moving_URTUU_TO_RZ_switch_mode_1(void)
{
	if((ROBOT_REAL_POS_DATA.X_POS > -1225) && (ROBOT_REAL_POS_DATA.Y_POS > 2000-DUAL_CENTER_DISTANCE)) return 1;
	else return 0;
}


// �ڶ�����������
u8 moving_URTUU_TO_RZ_switch_mode_2(void)    
{
	if((ROBOT_REAL_POS_DATA.X_POS < -1225) && (ROBOT_REAL_POS_DATA.Y_POS > 3500-DUAL_CENTER_DISTANCE)) return 1;
	else return 0;
}

// �������������ߣ�֮ǰԲ����һ�룩
u8 moving_URTUU_TO_RZ_switch_mode_3(void)
{
	if(ROBOT_REAL_POS_DATA.Y_POS > 5000-DUAL_CENTER_DISTANCE) return 1;
	else return 0;
}


// �����һ��������
u8 moving_URTUU_TO_RZ_switch_mode_4(void)
{
	if(ROBOT_REAL_POS_DATA.Y_POS > 5750-DUAL_CENTER_DISTANCE) return 1;
	else return 0;
}


// ����ڶ���������
u8 moving_URTUU_TO_RZ_switch_mode_5(void)
{
	if(ROBOT_REAL_POS_DATA.Y_POS > 6500-DUAL_CENTER_DISTANCE) return 1;
	else return 0;
}


// ���ŵ�ֱ��
u8 moving_URTUU_TO_RZ_switch_mode_6(void)
{
	if(ROBOT_REAL_POS_DATA.Y_POS > 8000) return 1;
	else return 0;
}


// ��������ֱ�ߵ�1/4Բ��
u8 moving_URTUU_TO_RZ_switch_mode_7(void)
{
	if(ROBOT_REAL_POS_DATA.X_POS < -1225-ARC_R_3) return 1;
	else return 0;
}


// ���һ��ֱ��
u8 moving_URTUU_TO_RZ_switch_mode_8(void)
{
	if(ROBOT_REAL_POS_DATA.X_POS < END_X) return 1;
	else return 0;
}


/*************************************************************************************/


// ֱ�߸���
void moving_line_track(void)
{
	float error;
	float vertical_unit_vector_x;
	float vertical_unit_vector_y;
	float	direct_unit_vector_x;
	float	direct_unit_vector_y;	
	float V_vertical;
	float V_direct;
	float V_parallel;
	float k1;
	float b1;
	float k2;
	float k3;
	float d;
	float b;

	// ���������λ�ô�ֱ��·�������������䵥λ����
	if(line.x1 != line.x0)  // ��·��б�ʴ���
	{
		k1 = (line.y1 - line.y0) / (line.x1 - line.x0);
		
		b1 = line.y0 - k1 * line.x0;
		
		if(k1 != 0)  // ��·��б�ʲ�Ϊ0
		{
			k2 = -1.0f / k1;
			
			vertical_unit_vector_x = sqrt(1.0f / (k2 * k2 + 1.0f));
			vertical_unit_vector_y = k2 * vertical_unit_vector_x;
			
			// ���������ȷ������ֵ
			if(k2 > 0)
			{
				if(mr1.y >= k1*mr1.x+b1)
				{
					error = -float_abs(-k1 * mr1.x + mr1.y + (line.y1 * line.x0 - line.y0 * line.x1) / (line.x1 - line.x0)) / sqrt(k1 * k1 + 1.0f);
				}
				else
				{
					error = float_abs(-k1 * mr1.x + mr1.y + (line.y1 * line.x0 - line.y0 * line.x1) / (line.x1 - line.x0)) / sqrt(k1 * k1 + 1.0f);					
				}
			}
			
			else
			{
				if(mr1.y >= k1*mr1.x+b1)
				{
					error = float_abs(-k1 * mr1.x + mr1.y + (line.y1 * line.x0 - line.y0 * line.x1) / (line.x1 - line.x0)) / sqrt(k1 * k1 + 1.0f);
				}
				else
				{
					error = -float_abs(-k1 * mr1.x + mr1.y + (line.y1 * line.x0 - line.y0 * line.x1) / (line.x1 - line.x0)) / sqrt(k1 * k1 + 1.0f);					
				}				
			}
		}
		
		else  // ��·��б��Ϊ0��ˮƽ��
		{
			vertical_unit_vector_x = 0;
			vertical_unit_vector_y = 1.0;		
			error = line.y0 - mr1.y;
		}
	}
	
	else  // ��·��б�ʲ����ڣ���ֱ��
	{
			vertical_unit_vector_x = 1.0;
			vertical_unit_vector_y = 0;		
			error = line.x0 - mr1.x;		
	}
	
	// �ٶȹ滮��Ҫ����
	d = sqrt((mr1.x - line.x1) * (mr1.x - line.x1) + (mr1.y - line.y1) * (mr1.y - line.y1));  // ������λ�õ�·���յ��ŷʽ����
	b = sqrt((d * d - error * error));                                                        // ������λ�õ�·���յ��ŷʽ��������·��ƽ�з���ķ���
	
	// �滮ƽ���ٶ�
	V_parallel = moving_line_velocity_planning(b);  // ***********************************************

	// ����pid���ƴ�ֱ������ٶ�
	// �ⲿ���Ƿ���ȡ����Ч�� 
//	if(float_abs(error) > line_pid.errormax) line_pid.errormax = float_abs(error);
//	line_pid.outputmax = float_abs(LINE_OUTPUTMAX_CONTROLLER * V_parallel * (error / ine_pid.errormax));
	line_pid.outputmax = float_abs(LINE_OUTPUTMAX_CONTROLLER * V_parallel);
	PID_incremental_PID_calculation_by_error(&line_pid, error);  
	V_vertical = line_pid.output;
	
	// ���������λ�õ�·���յ�ĵ�λ����
	if(line.x1 != mr1.x)  // ���λ�õ��յ��ֱ��б�ʴ���
	{
		k3 = (line.y1 - mr1.y) / (line.x1 - mr1.x);
		
		direct_unit_vector_x = sqrt(1.0f / (k3 * k3 + 1.0f));
		direct_unit_vector_y = k3 * direct_unit_vector_x;
		
		// ��ָ��·���յ㷽����ٶ�����ȷ������ֵ
		if(mr1.x > line.x1)
		{
			V_direct = -sqrt((error * V_parallel / b) * (error * V_parallel / b) + V_parallel * V_parallel);
		}
		
		else
		{
			V_direct = sqrt((error * V_parallel / b) * (error * V_parallel / b) + V_parallel * V_parallel);			
		}
	}
	
	else  // ���λ�õ��յ��ֱ��б�ʲ����ڣ���ֱ��
	{
		direct_unit_vector_x = 0;
		direct_unit_vector_y = 1;	
		
		// ��ָ��·���յ㷽����ٶ�����ȷ������ֵ		
		if(mr1.y > line.y1)
		{
			V_direct = -sqrt((error * V_parallel / b) * (error * V_parallel / b) + V_parallel * V_parallel);		
		}
		
		else if(mr1.y < line.y1)
		{
			V_direct = sqrt((error * V_parallel / b) * (error * V_parallel / b) + V_parallel * V_parallel);				
		}
		
		else
		{
			V_direct = 0.0f;					
		}
	}
	
	// ��������ٶ�
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = MM_PER_SEC_TO_RPM * (V_vertical * vertical_unit_vector_x + V_direct * direct_unit_vector_x);
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = MM_PER_SEC_TO_RPM * (V_vertical * vertical_unit_vector_y + V_direct * direct_unit_vector_y);
}


// Բ������
void moving_arc_track(void)
{
	float distance;
	float error;
	float center_unit_vector_x;
	float center_unit_vector_y;
	float	tangent_unit_vector_x;
	float	tangent_unit_vector_y;	
	float V_center;
	float V_tangent;
	float k1;
	float k2;
	
	// ����error�뵥λ����
	distance = sqrt((mr1.y - arc.y) * (mr1.y - arc.y) + (mr1.x - arc.x) * (mr1.x - arc.x));
	
	if (mr1.x != arc.x) // λ�õ�Բ��ֱ��б�ʴ���
	{
		// ����б��
		k1 = (mr1.y - arc.y) / (mr1.x - arc.x);
		
		// �õ�λ����ָ��Բ��
		if(mr1.x > arc.x)
		{
			center_unit_vector_x = -sqrt(1.0f / (k1 * k1 +1.0f));
			center_unit_vector_y = k1 * center_unit_vector_x;
		}
		
		else
		{
			center_unit_vector_x = sqrt(1.0f / (k1 * k1 +1.0f));
			center_unit_vector_y = k1 * center_unit_vector_x;			
		}
		
		// ��error����ȷ������ֵ
		if(distance < arc.R)
		{
			error = -float_abs(arc.R - distance);
		}
		
		else
		{
			error = float_abs(arc.R - distance);
		}
	}
	
	else // λ�õ�Բ��ֱ��б�ʲ����ڣ���ֱ��
	{
		// �õ�λ����ָ��Բ��		
		if(mr1.y > arc.y)
		{
			center_unit_vector_x = 0;
			center_unit_vector_y = -1.0;		
		}
		
		else
		{
			center_unit_vector_x = 0;
			center_unit_vector_y = 1.0;				
		}
		
		// ��error����ȷ������ֵ
		if(distance < arc.R)
		{
			error = -float_abs(arc.R - distance);
		}
		
		else
		{
			error = float_abs(arc.R - distance);			
		}
	}
	
	// �������߷���ĵ�λ���� ����ָ��˳ʱ�뷽��
	// �ٶ�Ϊ��ֵ ˳ʱ���˶� 
	// �ٶ�Ϊ��ֵ ��ʱ���˶�
	if(mr1.x != arc.x) // λ�õ�Բ��ֱ��б�ʴ���
	{
		if(k1 != 0)  // k2����
		{
			k2 = -1.0 / k1;
			
			if(mr1.y > arc.y)
			{
				tangent_unit_vector_x = sqrt(1.0f / (k2 * k2 + 1.0f));
				tangent_unit_vector_y = k2 * tangent_unit_vector_x;
			}
			
			else
			{
				tangent_unit_vector_x = -sqrt(1.0f / (k2 * k2 + 1.0f));
				tangent_unit_vector_y = k2 * tangent_unit_vector_x;				
			}
		}
		
		else  // k2�����ڣ�ˮƽ��
		{
			if(mr1.x > arc.x)
			{
				tangent_unit_vector_x = 0;
				tangent_unit_vector_y = -1;			
			}
			
			else
			{
				tangent_unit_vector_x = 0;
				tangent_unit_vector_y = 1;			
			}			
		}
	}
	else // λ�õ�Բ��ֱ��б�ʲ�����
	{
		if(mr1.y > arc.y)
		{
			tangent_unit_vector_x = 1;
			tangent_unit_vector_y = 0;			
		}
		
		else
		{
			tangent_unit_vector_x = -1;
			tangent_unit_vector_y = 0;			
		}
	}

	// �滮���ٶ� 
	V_tangent = moving_arc_velocity_planning(-center_unit_vector_x, -center_unit_vector_y);

	// ����pid����Բ�ķ�����ٶ�
	// �ⲿ���Ƿ���ȡ����Ч�� 
//	if(float_abs(error) > arc_pid.errormax) arc_pid.errormax = float_abs(error);
//	arc_pid.outputmax = float_abs(ARC_OUTPUTMAX_CONTROLLER * V_tangent * (error / arc_pid.errormax));
	arc_pid.outputmax = float_abs(ARC_OUTPUTMAX_CONTROLLER * V_tangent);
	PID_incremental_PID_calculation_by_error(&arc_pid, error);   
	V_center = arc_pid.output;

	// ��������ٶ�
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = MM_PER_SEC_TO_RPM * (V_center * center_unit_vector_x + V_tangent * tangent_unit_vector_x);
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = MM_PER_SEC_TO_RPM * (V_center * center_unit_vector_y + V_tangent * tangent_unit_vector_y);
}


// �����
void moving_point_track(void)
{
	float x_error;
	float y_error;
	float Vx;
	float Vy;
	
	// �������
	x_error = point.x - mr1.x;
	y_error = point.y - mr1.y;
	
	// ����X�᷽���ٶ�
	// �ⲿ���Ƿ���ȡ����Ч�� 
//	if(float_abs(x_error) > point_x_pid.errormax) point_x_pid.errormax = float_abs(x_error);
//	point_x_pid.outputmax = float_abs(POINT_X_OUTPUTMAX_CONTROLLER * point.Vmax * (x_error / point_x_pid.errormax));
	point_x_pid.outputmax = float_abs(POINT_X_OUTPUTMAX_CONTROLLER * point.Vmax);
	PID_incremental_PID_calculation_by_error(&point_x_pid, x_error);   
	Vx = point_x_pid.output;
	
	// ����X�᷽���ٶ�
	// �ⲿ���Ƿ���ȡ����Ч�� 
//	if(float_abs(y_error) > point_y_pid.errormax) point_y_pid.errormax = float_abs(y_error);
//	point_y_pid.outputmax = float_abs(POINT_Y_OUTPUTMAX_CONTROLLER * point.Vmax * (y_error / point_y_pid.errormax));
	point_y_pid.outputmax = float_abs(POINT_Y_OUTPUTMAX_CONTROLLER * point.Vmax);
	PID_incremental_PID_calculation_by_error(&point_y_pid, y_error);   
	Vy = point_y_pid.output;
	
	// ��������ٶ�
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = MM_PER_SEC_TO_RPM * Vx;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = MM_PER_SEC_TO_RPM * Vy;	
}


/***************************************************************************/


// �߸����ٶȹ滮
// �����˶���line.Vstart�����0
// ���μӼ����˶����ٶȴ�С�����ٶ�����Ϲ���
float moving_line_velocity_planning(float b)
{
	float V_planned = 0.0f;
	
	switch(NOW_PATH[PATH_POINT_COUNTER].velocity_planning_type)
	{
		case 0 : V_planned = float_abs(line.Vstart); break;                          // ����
		case 1 : V_planned = moving_line_velocity_planning_trapezoidal_V(b); break;  // ���μӼ���
		default : break;
	}
	
	return V_planned;
}

// �߸����ٶȹ滮-���μӼ���
// b>traV.s_totalʱ V_traV=line.Vstart
// b<traV.s_totalʱ V_traV��·���жϹ�����
float moving_line_velocity_planning_trapezoidal_V(float b)
{
	float V_traV = 0.0f;
	float s;
	
	s = traV.s_total - b;  // ����ƽ�з����߹���·��
	
	// ���ٶȸ�ֵ
	if((line.Vmax >= line.Vstart) && (line.Vmax >= line.Vend) && (line.a1 >= 0) && (line.a2 <= 0)  && (line.Vstart >= 0) && (line.Vmax >= 0) && (line.Vend >= 0))  // ����������
	{
		if((traV.s_total >= (traV.s_ac + traV.s_de)) && (s >= 0))  // ��·����Դ�С����
		{
			if(s < traV.s_ac) V_traV = sqrt(2.0f * line.a1 * s + line.Vstart * line.Vstart);  // ���ٽ׶�
			else if(s < traV.s_ac+traV.s_co) V_traV = line.Vmax;                              // ���ٽ׶�
			else V_traV = sqrt(line.Vend * line.Vend - 2.0f * line.a2 * b);                   // ���ٽ׶�
		}
		
		else  // ��·����Դ�С������
		{
			V_traV = float_abs(line.Vstart);
		}
	}
	
	else  // ������������
	{
		V_traV = float_abs(line.Vstart);
	}
	
	return V_traV;
}


// Բ�������ٶȹ滮
// �����˶���line.Vstart����0 ˳ʱ���˶� line.VstarС��0 ��ʱ���˶�
// ���μӼ����˶����ٶȴ�С�����ٶ�����Ϲ���
float moving_arc_velocity_planning(float loc_vector_x, float loc_vector_y)
{
	float V_planned = 0.0f;

	switch(NOW_PATH[PATH_POINT_COUNTER].velocity_planning_type)
	{
		case 0 : V_planned = arc.Vstart; break;                                    // ����
		case 1 : V_planned = moving_arc_velocity_planning_trapezoidal_V(loc_vector_x, loc_vector_y); break;  // ���μӼ���
		default : break;
	}
	
	return V_planned;
}


// Բ�������ٶȹ滮-���μӼ���
float moving_arc_velocity_planning_trapezoidal_V(float loc_vector_x, float loc_vector_y)
{
	float V_traV = 0.0f;
	float rad;
	float vector_product;
	int clockwise_direction;
	float s;
	
	// ����н�
	rad = acos(loc_vector_x * traV.sita_vector_x + loc_vector_y * traV.sita_vector_y);
	
	// ���ò���ж��������λ��
	vector_product = traV.sita_vector_x * loc_vector_y - traV.sita_vector_y * loc_vector_x;
	
	// �ж��ƶ������Ƿ����
	if(vector_product != 0)
	{
		if(vector_product > 0)
		{	
			clockwise_direction = -1;  // λ��-Բ����������ʼ��-Բ��������˳ʱ��ǰ��
		}
		
		else
		{
			clockwise_direction = 1;  // λ��-Բ����������ʼ��-Բ����������ʱ��ǰ��
		}
		
		if(arc.d_sita*clockwise_direction > 0)  // ��·����
		{
			s = rad * arc.R;
		}
		
		else  // ��δ�������
		{
			s = 0.0f;
		}
	}
	
	else
	{
		if(rad == 0)  // �պ������
		{
			s = 0.0f;
		}
		else // ����0-PI�ķ�Χ
		{
			s = traV.s_total;
		}
	}
	
	// ��ֹ����
	if(s > traV.s_total) s = traV.s_total;  // ���յ����
	
	// ���ٶȸ�ֵ
	if((arc.Vmax >= arc.Vstart) && (arc.Vmax >= arc.Vend) && (arc.a1 >= 0) && (arc.a2 <= 0)  && (arc.Vstart >= 0) && (arc.Vmax >= 0) && (arc.Vend >= 0))  // ����������
	{
		if((traV.s_total >= (traV.s_ac + traV.s_de)) && (s >= 0))  // ��·����Դ�С����
		{
			if(s < traV.s_ac) V_traV = sqrt(2.0f * arc.a1 * s + arc.Vstart * arc.Vstart);  // ���ٽ׶�
			else if(s < traV.s_ac+traV.s_co) V_traV = arc.Vmax;                            // ���ٽ׶�
			else V_traV = sqrt(arc.Vend * arc.Vend - 2.0f * arc.a2 * (traV.s_total - s));  // ���ٽ׶�
		}
		
		else  // ��·����Դ�С������
		{
			V_traV = arc.Vstart;
		}
	}
	
	else  // ������������
	{
		V_traV = arc.Vstart;
	}
	
	return V_traV;
}


/*************************************************************************************/


/*WAITING_TEST*/
// ��PID����ƫ����
void moving_yaw_adjusting_pure_PID(void)
{
	float W;
	float error;
	
	// �������
	if(mr1.yaw*yaw.yaw >= 0)
	{
		error = yaw.yaw - mr1.yaw;
	}
	
	else
	{
		if(float_abs(mr1.yaw)+float_abs(yaw.yaw) <= 180) error = yaw.yaw - mr1.yaw;
		else 
		{
			if((yaw.yaw-mr1.yaw) > 0) 
			{
				error = yaw.yaw - mr1.yaw - 360;
			}
			else if((yaw.yaw-mr1.yaw) < 0)
			{
				error = yaw.yaw - mr1.yaw + 360;
			}
		}
	}
	
  // ת��Ϊ����
	error = error / 180.0f * PI;

	// ֱ������PID������ٶ�
	PID_incremental_PID_calculation_by_error(&yaw_pid, error); 
	
	W = yaw_pid.output;	  // ���̽��ٶ� ��λ��rad/s
	
	ROBOT_TARGET_VELOCITY_DATA.W_RPM = MM_PER_SEC_TO_RPM * W * CHASSIS_RADIUS;  /*WAITING_TEST*/
}


// ʱ��Լ����PID����ƫ����
void moving_yaw_adjusting_time_and_PID(void)
{
 	float W;
	float error; 
	float passed_time;
	static u8 cal_W_flag = 1;
	
	// ��ֹ����
	if(yaw.yaw_t1 >= yaw.yaw_t2)
	{
		ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
		
		return;
	}
	
	// ���㾭����ʱ��
	passed_time = YAW_TIME_COUNTER * TRACK_RUN_PERIOD;
	
	// ͨ��ʱ���ж�
	if(passed_time <= yaw.yaw_t1)
	{
		// ������
		ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
		
		cal_W_flag = 1;
	}
	
	else if((passed_time <= yaw.yaw_t2) && (cal_W_flag == 1))
	{
		// ֱ�Ӽ�����ٶȵ���
		// �������
		if(mr1.yaw*yaw.yaw >= 0)
		{
			error = yaw.yaw - mr1.yaw;
		}
		
		else
		{
			if(float_abs(mr1.yaw)+float_abs(yaw.yaw) <= 180) error = yaw.yaw - mr1.yaw;
			else 
			{
				if((yaw.yaw-mr1.yaw) > 0) 
				{
					error = yaw.yaw - mr1.yaw - 360;
				}
				else if((yaw.yaw-mr1.yaw) < 0)
				{
					error = yaw.yaw - mr1.yaw + 360;
				}
			}
		}
		
		error = error / 180.0f * PI;
		
		W = error / (yaw.yaw_t2 - yaw.yaw_t1);
		
		ROBOT_TARGET_VELOCITY_DATA.W_RPM = MM_PER_SEC_TO_RPM * W * CHASSIS_RADIUS;
		
		cal_W_flag = 0;  // һ��·����ֻ�������һ��
	}
	
	else
	{
		// PID����
		moving_yaw_adjusting_pure_PID();
	}

	YAW_TIME_COUNTER ++;
}








