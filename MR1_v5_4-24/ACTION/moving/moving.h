#ifndef __MOVING_H
#define __MOVING_H
#include "stm32f4xx.h"

#define absolute(x) ((x)>0? (x):-(x))

// 各个PID的outputmax乘上的系数
#define LINE_OUTPUTMAX_CONTROLLER    0.0f
#define ARC_OUTPUTMAX_CONTROLLER     0.0f
#define YAW_OUTPUTMAX_CONTROLLER     0.0f
#define POINT_X_OUTPUTMAX_CONTROLLER 1.0f
#define POINT_Y_OUTPUTMAX_CONTROLLER 1.0f

// 底盘半径
#define CHASSIS_RADIUS 350.0f //**************************************************需修改

// 调整底盘速度函数的运行周期
#define TRACK_RUN_PERIOD 1  // 单位：ms

// 计算公式：1/（pi*轮子直径）*减速比*60
#define MM_PER_SEC_TO_RPM 2.639959f   //轮子直径152mm，电机减速比1:21，轮子一圈pi*152mm

typedef struct PLANNING_PATH
{
	u8 track_type;
	u8 velocity_planning_type;
	u8 yaw_adjusting_type;
	float x0_or_xcenter;
	float y0_or_ycenter;
	float x1_or_R;
	float y1_or_none;
	float Vstart;
	float a1;
	float Vmax;
	float a2;
	float Vend;
	float sita;
	float d_sita;
	float yaw_t1;
	float yaw_t2;
	float yaw;
}PLANNING_PATH;

typedef struct arc_struct
{
	float x;
	float y;
	float R;
	float Vstart;
	float a1;
	float Vmax;
	float a2;
	float Vend;
	float sita;  // 范围：(-PI, PI]
	float d_sita;
	float yaw_t1;
	float yaw_t2;
	float yaw;
}arc_struct;

typedef struct line_struct
{
	float x0;
	float y0;
	float x1;
	float y1;
	float Vstart;
	float a1;
	float Vmax;
	float a2;
	float Vend;
	float yaw_t1;  // 单位：s
	float yaw_t2;  // 单位：s
	float yaw;
}line_struct;

typedef struct point_struct
{
	float x;
	float y;
	float Vmax;
}point_struct;

typedef struct robot_struct
{
	float x;
	float y;
	float yaw;
}robot_struct;

typedef struct trapezoidal_V_struct
{
	float s_total;
	float s_ac;
	float s_de;
	float s_co;
	float sita_vector_x;
	float sita_vector_y;
}trapezoidal_V_struct;

typedef struct yaw_struct
{
	float yaw_t1;
	float yaw_t2;
	float yaw;
}yaw_struct;

extern enum MOVING_STATE_ITEMS MOVING_STATE;
extern struct ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
extern struct PLANNING_PATH *NOW_PATH;
extern struct PLANNING_PATH DEBUG_PLANNING_PATH[];

float float_abs(float value);
void moving_action_task(void);
void moving_init(void);
void moving_path_tracking(void);
void moving_assign_path_parameter(void);
void moving_assign_velocity_planning_parameter(void);
void moving_arc_track(void);
void moving_line_track(void);
void moving_point_track(void);
void moving_select_switch_mode_URTUU_TO_RZ(void); 
u8 moving_URTUU_TO_RZ_switch_mode_0(void);
u8 moving_URTUU_TO_RZ_switch_mode_1(void);
u8 moving_URTUU_TO_RZ_switch_mode_2(void);
u8 moving_URTUU_TO_RZ_switch_mode_3(void);
u8 moving_URTUU_TO_RZ_switch_mode_4(void);
u8 moving_URTUU_TO_RZ_switch_mode_5(void);
u8 moving_URTUU_TO_RZ_switch_mode_6(void);
u8 moving_URTUU_TO_RZ_switch_mode_7(void);
u8 moving_URTUU_TO_RZ_switch_mode_8(void);
float moving_line_velocity_planning(float b);
float moving_line_velocity_planning_trapezoidal_V(float b);
float moving_arc_velocity_planning(float loc_unit_vector_x, float loc_unit_vector_y);
float moving_arc_velocity_planning_trapezoidal_V(float loc_unit_vector_x, float loc_unit_vector_y);
void moving_yaw_adjusting_pure_PID(void);
void moving_yaw_adjusting_time_and_PID(void);

#endif
