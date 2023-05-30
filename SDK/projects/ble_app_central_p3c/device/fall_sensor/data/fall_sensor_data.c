
#include "fall_sensor_data.h"

HumanActivityInfo fall_sensor_motion_data;		//跌倒传感器之加速度传感器部分的数据

unsigned char p3c_motion_sensor_state 		= 0;
unsigned char p3c_pressure_sensor_state 	= 0;
unsigned char tdun_fall_state				= 0;//motion sensor fall state
unsigned char tdun_move_state				= 0;//当前的运动状态 0 表示静止，1 表示慢走，2 表示快走，3 表示慢跑，4 表示快跑
unsigned char z_num 						= 0xf0;
unsigned short sensor_x						= 0;
unsigned short sensor_y						= 0;
unsigned short sensor_z						= 0;
unsigned int sensor_cnt 					= 0;
unsigned int tdun_step_num					= 0;//motion sensor step num
unsigned int move_ticks						= 0;//运动时长 单位：秒
////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned char get_motion_sensor_state(void)
{
	return p3c_motion_sensor_state;
}

void set_motion_sensor_state(unsigned char state)
{
	p3c_motion_sensor_state = state;
}
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned char get_pressure_sensor_state(void)
{
	return p3c_pressure_sensor_state;
}

void set_pressure_sensor_state(unsigned char state)
{
	p3c_pressure_sensor_state = state;
}
////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned char get_move_ticks_rec(void)
{
	return move_ticks;
}

void set_move_ticks_rec(unsigned int tick)
{
	move_ticks = tick;
}
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned char get_fall_state(void)
{
	return tdun_fall_state;
}

void set_fall_state(unsigned char state)
{
	tdun_fall_state = state;
}

void reset_fall_state(void)
{
	tdun_fall_state = 0;
}
////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned int get_steps(void)
{
	return tdun_step_num;
}
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************
*
*
***********************************************************/
unsigned char get_move_state(void)
{
	return tdun_move_state;
}
////////////////////////////////////////////////////////////////////////////////////////


