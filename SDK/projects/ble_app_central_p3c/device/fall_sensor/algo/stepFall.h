#ifndef	_STEPFALL_H_
#define	_STEPFALL_H_

#include "commonALG.h"

typedef struct {
	TUInt8	fall_flag;
	TUInt8	sedentary_flag;
	TUInt8	walk_run_status;
	TUInt8	wear_flag;
	TUInt32	steps;



	float averFrequercy;
	float averAmp;
}HumanActivityInfo,*PHumanActivityInfo;

typedef struct {
	TInt32 sens;
	TInt32 d1;
	TInt32 d2;
	TInt32 d3;
	TInt32 d4;
	TInt32 d5;
	TInt32 d6;
	TInt32 d7;
	TInt32 d8;
	TInt32 d9;
	TInt32 d10;

}DEBUGINFOR, *PDEBUGINFO;


void set_dect_step_threshold(float maxHz, float minHz, float amplitude);
void set_judge_walking_threshold(float startTime, float endTime, TInt32 judgesteps);
void set_wearthreshold(TInt32 minutetime, float move_g);
void set_sedentarythreshold(TUInt16 minutetime);
void set_walkstatusthreshold(TInt32 mseconds);
void set_acc_sens(TInt32 sens);

void init_judege_walking(TUInt8 flg_init_step);

void ALPSLIB_Init(void);
HumanActivityInfo ALPSLIB_PutData_Time(TInt16 acc_x, TInt16 acc_y, TInt16 acc_z, TUInt32 pressure, TUInt16 w_year, TUInt8 w_month, TUInt8 w_date, TUInt8 hour, TUInt8 min, TUInt8 sec);
void clear_sedentary_flag(void);
void clear_wear_flag(void);
PDEBUGINFO get_alg_information(void);

#endif

