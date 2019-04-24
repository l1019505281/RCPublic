#include "main.h"


ENCO_DIS ENCO_DIS_DATA = {0, 0, 0, 0};
GL_POS GL_POS_DATA = {0, 0, 0};


// 初始化全场定位模块
void gl_GL_model_init(void)
{
	u8 time_counter = 0;
	
	// 初始化底层配置
	encoder_TIM4_init();
  encoder_TIM8_init();
	gyro_UART4_init(115200);
	
	// 等待陀螺仪启动与发送数据 读取陀螺仪Z轴初始角度
	// 在主循环开始前延时，不影响时序
	while(time_counter < 100)
	{
		time_counter ++;
		delay_ms(5);
	}
	
	// 设置位置偏置量 （这里设机器人启动位置为原点）*************************需改到FSM中选择模式括号内
	ROBOT_POS_OFFSET_DATA.X_CS_OFFSET = -500.0;
	ROBOT_POS_OFFSET_DATA.Y_CS_OFFSET =  500.0;
	ROBOT_POS_OFFSET_DATA.YAW_CS_OFFSET = 0.0;
	
	GL_POS_DATA.X = ROBOT_POS_OFFSET_DATA.X_CS_OFFSET;
	GL_POS_DATA.Y = ROBOT_POS_OFFSET_DATA.Y_CS_OFFSET;
	
	// 获取初始偏航角
	ROBOT_POS_OFFSET_DATA.YAW_IN_OFFSET = -(float)GYRO_ANGLE.Angle[2] / 32768 * 180;  
}


// 更新全场定位定位位置数据
void gl_update_GL_position(void)
{
	// 读取定时器CNT寄存器里面的值
	// 朝外运动，值应该为正，根据这个调整正负号
	ENCO_DIS_DATA.CNT_X =  -encoder_get_encoder_CNT(4);  // 接顺时针首轮子
	ENCO_DIS_DATA.CNT_Y = -encoder_get_encoder_CNT(8);   // 接顺时针尾轮子

	// 计算编码器行进的路程值
	ENCO_DIS_DATA.DELTA_DIS_X = ENCO_DIS_DATA.CNT_X * DISTANCE_PER_CNT;
	ENCO_DIS_DATA.DELTA_DIS_Y = ENCO_DIS_DATA.CNT_Y * DISTANCE_PER_CNT;
	
	// 将编码器行进的路程值转化全场定位测量出的位置
	gl_encoder_to_GL_conversion();
}


// 将编码器行进的路程值转化全场定位测量出的位置
void gl_encoder_to_GL_conversion(void)
{
	static float last_yaw = 0;  // 上一次的偏航角
	float delta_rad = 0;  // 偏航角弧度差
	float model_yaw = 0;  // 模块坐标系偏航角
		
	// 直接获取偏航角，并进行简单处理（偏航角度值全是逆时针为正，顺时针为负）
	GL_POS_DATA.YAW = (float)GYRO_ANGLE.Angle[2] / 32768 * 180 - ROBOT_POS_OFFSET_DATA.YAW_IN_OFFSET + ROBOT_POS_OFFSET_DATA.YAW_CS_OFFSET;  /*WAITING_TEST*/
	// 限制范围(-180， 180]
	if(GL_POS_DATA.YAW > 180) {GL_POS_DATA.YAW = -180 + (GL_POS_DATA.YAW - 180);}
	else if(GL_POS_DATA.YAW <= -180) {GL_POS_DATA.YAW = 180 + (GL_POS_DATA.YAW + 180);}
	
	// 计算模块坐标系偏航角
	model_yaw = GL_POS_DATA.YAW + YAW0;
	// 限制范围(-180， 180]	
	if(model_yaw > 180) {model_yaw = -180 + (model_yaw - 180);}
	else if(model_yaw <= -180) {model_yaw = 180 + (model_yaw + 180);}
	
	// 计算偏航角弧度差并保存当前偏航角
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
	
	// 运动学方程
	GL_POS_DATA.X += (ENCO_DIS_DATA.DELTA_DIS_X - Lx * delta_rad) * cos(model_yaw / 180.0f * PI) - 
									 (ENCO_DIS_DATA.DELTA_DIS_Y + Ly * delta_rad) * sin(model_yaw / 180.0f * PI);
	
	GL_POS_DATA.Y += (ENCO_DIS_DATA.DELTA_DIS_X - Lx * delta_rad) * sin(model_yaw / 180.0f * PI) + 
	                 (ENCO_DIS_DATA.DELTA_DIS_Y + Ly * delta_rad) * cos(model_yaw / 180.0f * PI);

}








