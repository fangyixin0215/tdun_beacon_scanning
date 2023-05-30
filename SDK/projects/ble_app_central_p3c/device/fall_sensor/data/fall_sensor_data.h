#ifndef __FALL_SENSOR_DATA_H__
#define __FALL_SENSOR_DATA_H__
#include "stepFall.h"

extern HumanActivityInfo fall_sensor_motion_data;		//跌倒传感器之加速度传感器部分的数据
extern unsigned char p3c_motion_sensor_state;
extern unsigned char p3c_pressure_sensor_state;
extern unsigned char tdun_fall_state;//motion sensor fall state
extern unsigned char tdun_move_state;//motion sensor move state
extern unsigned char z_num;
extern unsigned short sensor_x;
extern unsigned short sensor_y;
extern unsigned short sensor_z;
extern unsigned int sensor_cnt;
extern unsigned int tdun_step_num;//motion sensor step num

unsigned char get_motion_sensor_state(void);
unsigned char get_pressure_sensor_state(void);
unsigned char get_fall_state(void);
unsigned char get_move_state(void);
unsigned char get_move_ticks_rec(void);
unsigned int get_steps(void);


void set_move_ticks_rec(unsigned int tick);
void set_motion_sensor_state(unsigned char state);
void set_pressure_sensor_state(unsigned char state);
void set_fall_state(unsigned char state);
void reset_fall_state(void);

#endif
