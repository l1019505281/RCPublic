#include "main.h"


ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0, 0, 0, 0};
ROBOT_POS_OFFSET ROBOT_POS_OFFSET_DATA;


// ��ȡ�����˵���ʵλ��
void location_update_robot_real_position(void)
{
	// ��¼�ϴε�λ��
	ROBOT_REAL_POS_DATA.LAST_X_POS = ROBOT_REAL_POS_DATA.X_POS;
	ROBOT_REAL_POS_DATA.LAST_Y_POS = ROBOT_REAL_POS_DATA.Y_POS;
	ROBOT_REAL_POS_DATA.LAST_YAW_POS = ROBOT_REAL_POS_DATA.YAW_POS;
	 
	// ����ȫ����λ����
	gl_update_GL_position();
	
	// ��ֵ����ʵλ������
	ROBOT_REAL_POS_DATA.X_POS = GL_POS_DATA.X;
	ROBOT_REAL_POS_DATA.Y_POS = GL_POS_DATA.Y;
	ROBOT_REAL_POS_DATA.YAW_POS = GL_POS_DATA.YAW;	
}
