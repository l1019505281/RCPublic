#include "main.h"


ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0, 0, 0, 0};
ROBOT_POS_OFFSET ROBOT_POS_OFFSET_DATA;


// 获取机器人的真实位置
void location_update_robot_real_position(void)
{
	// 记录上次的位置
	ROBOT_REAL_POS_DATA.LAST_X_POS = ROBOT_REAL_POS_DATA.X_POS;
	ROBOT_REAL_POS_DATA.LAST_Y_POS = ROBOT_REAL_POS_DATA.Y_POS;
	ROBOT_REAL_POS_DATA.LAST_YAW_POS = ROBOT_REAL_POS_DATA.YAW_POS;
	 
	// 更新全场定位数据
	gl_update_GL_position();
	
	// 赋值给真实位置数据
	ROBOT_REAL_POS_DATA.X_POS = GL_POS_DATA.X;
	ROBOT_REAL_POS_DATA.Y_POS = GL_POS_DATA.Y;
	ROBOT_REAL_POS_DATA.YAW_POS = GL_POS_DATA.YAW;	
}
