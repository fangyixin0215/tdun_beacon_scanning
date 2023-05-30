/**
****************************************************************************************
*
* @file icu.c
*
* @brief icu initialization and specific functions
*
* Copyright (C) Beken 2009-2016
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup ICU
* @ingroup ICU
* @brief ICU
*
* This is the driver block for ICU
* @{
****************************************************************************************
*/


#include <stddef.h>     // standard definition
#include "rwip_config.h"
#include "rwip.h"
#include "user_config.h"
#include "BK3435_reg.h"
#include "icu.h"      // timer definition
#include "rf.h"
#include "flash.h"
#include "uart.h"


static uint8_t default_sleep_mode = 0;  //0:Ωµ—π–›√ﬂ   1:∆’Õ®idel
extern uint8_t system_sleep_flag;
extern uint8_t system_mode;
extern uint32_t flash_mid;

extern volatile uint32_t XVR_ANALOG_REG_BAK[16];

void system_sleep_init(void)
{
#if SYSTEM_SLEEP
    system_sleep_flag = 0x1;
#else
    system_sleep_flag = 0x0;
#endif
    if((system_mode & RW_DUT_MODE) == RW_DUT_MODE)
    {
		system_sleep_flag = 0x0;
    }
}


void icu_init(void)
{
#if DEBUG_HW
    REG_AHB0_ICU_LPO_CLK_ON |= (0x01 << 2);
#endif

    REG_AHB0_ICU_FLASH = 0;
    REG_AHB0_ICU_FLASH |= (0x25 << 24);     
    REG_AHB0_ICU_FLASH |= (0x15 << 16);
    REG_AHB0_ICU_FLASH |= (0x10f << 0);

    if( (flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
    {
        REG_AHB0_ICU_CPU_STATUS  = 0x661;
        REG_AHB0_ICU_CLKSRCSEL = 0X000001B1; //usr 16M,
    }
    else
    {
        REG_AHB0_ICU_CPU_STATUS  = 0x771;
        REG_AHB0_ICU_CLKSRCSEL = 0X000001F9; //usr 16M,
    }


    REG_AHB0_ICU_DIGITAL_PWD = REG_AHB0_ICU_DIGITAL_PWD & (~0X02);

    REG_AHB0_ICU_CORECLKCON = 0X00;

        

    REG_AHB0_ICU_ANA_CTL |= (0X01 << 6);

    switch_clk(MCU_DEFAULT_CLK);

}


uint8_t icu_get_sleep_mode(void)
{
	return default_sleep_mode;
}


void icu_set_sleep_mode(uint8_t v)
{
	default_sleep_mode = v;
}
#if SYSTEM_SLEEP


void cpu_reduce_voltage_sleep(void)
{
    uint32_t tmp_reg;
    uint8_t calc_num; 

    calc_num = 0;

    REG_AHB0_ICU_DIGITAL_PWD |= 0x80 ;
    
    tmp_reg=REG_AHB0_ICU_DIGITAL_PWD|0X0F;

#if(HZ32000)
    XVR_REG09 = XVR_ANALOG_REG_BAK[0x09] &(~(0x01 << 26));
#else
    XVR_REG09 = XVR_ANALOG_REG_BAK[0x09]|(0x01 << 26);
#endif     
 
#if (REDUCE_VOL_SLEEP)
    REG_AHB0_ICU_INT_ENABLE &= (~(0x01 << 9));

    if( (flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
    {
        REG_AHB0_ICU_CLKSRCSEL = 0x19C;
        REG_AHB0_ICU_CPU_STATUS  = 0x631;
    }
    else
    {
        REG_AHB0_ICU_CLKSRCSEL = 0x1DC;
        REG_AHB0_ICU_CPU_STATUS  = 0x731;
    }
        
#else
     if((flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
    {
        REG_AHB0_ICU_CLKSRCSEL = 0X000001B0; //usr 16M,
    }
    else
    {
        REG_AHB0_ICU_CLKSRCSEL = 0X000001F8; //usr 16M,
    }
#endif

    REG_AHB0_ICU_DIGITAL_PWD =tmp_reg;

    REG_AHB0_ICU_CORECLKCON  = 0x1; // Power down MCU

    ////wakeup   
#if DEBUG_HW
    REG_AHB0_ICU_DIGITAL_PWD = 0x80 | (0X01 << 4);
#else
    REG_AHB0_ICU_DIGITAL_PWD = 0x80 ;
#endif
    while(calc_num++ < 30) //delay 22us        
    {                
    	//__nop();        
	}

}

void cpu_wakeup(void)
{	    
    
#if (REDUCE_VOL_SLEEP)
    REG_AHB0_ICU_INT_ENABLE |= (0x01 << 9);

    if( (flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
    {
        REG_AHB0_ICU_CPU_STATUS  = 0x661;
        REG_AHB0_ICU_CLKSRCSEL = 0X000001B1; //usr 16M,
    }
    else
    {
        REG_AHB0_ICU_CPU_STATUS  = 0x771;
        REG_AHB0_ICU_CLKSRCSEL = 0X000001F9; //usr 16M,
    }        
#else
     if( (flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
    {
        REG_AHB0_ICU_CLKSRCSEL = 0X000001B1; //usr 16M,
    }
    else
    {
        REG_AHB0_ICU_CLKSRCSEL = 0X000001F9; //usr 16M,
    }
#endif
    if(MCU_DEFAULT_CLK!=MCU_CLK_16M)
        switch_clk(MCU_DEFAULT_CLK);
    
}

void cpu_idle_sleep(void)
{
	REG_AHB0_ICU_CORECLKCON = 0x01;//MCU sleep
}

#endif




void switch_clk(uint8_t clk)
{
    REG_AHB0_ICU_DIGITAL_PWD = REG_AHB0_ICU_DIGITAL_PWD & (~0X02);

    if (clk == MCU_CLK_16M )
    {
        XVR_ANALOG_REG_BAK[0x09] &= (~((0X01<<6)|(0X01<<26)));
        #if (!HZ32000)        
        XVR_ANALOG_REG_BAK[0x09] |= (0X01<<26);
        #endif
        mHWreg_Assign_XVR_Regsiter(09, XVR_ANALOG_REG_BAK[0x09]);

        REG_AHB0_ICU_CORECLKCON = 0X00; //clk div 0

        if( (flash_mid==XTX_FLASH_1)&&(HZ32000==1) )
        {
            REG_AHB0_ICU_CLKSRCSEL = 0X000001B1; //usr 16M,
        }
        else
        {
            REG_AHB0_ICU_CLKSRCSEL = 0X000001F9; //usr 16M,
        }

    }
    else if (clk == MCU_CLK_64M )
    {
        XVR_ANALOG_REG_BAK[0x09] |= ((0X01<<6)|(0X01<<26));
        #if (HZ32000)        
        XVR_ANALOG_REG_BAK[0x09] &= (~(0X01<<26));
        #endif
        mHWreg_Assign_XVR_Regsiter(09, XVR_ANALOG_REG_BAK[0x09]);

        REG_AHB0_ICU_CORECLKCON = 0X00; //clk div 0

        REG_AHB0_ICU_CLKSRCSEL = 0X000005FB; //usr PLL CLK SELT 64M

    }
    else if (clk == MCU_CLK_48M )
    {
        XVR_ANALOG_REG_BAK[0x09] &= (~((0X01<<6)|(0X01<<26)));
        #if (!HZ32000)        
        XVR_ANALOG_REG_BAK[0x09] |= (0X01<<26);
        #endif
        mHWreg_Assign_XVR_Regsiter(09, XVR_ANALOG_REG_BAK[0x09]);
        
        REG_AHB0_ICU_CORECLKCON = 0x04; //clk div 2

        REG_AHB0_ICU_CLKSRCSEL = 0X000001FB; //usr PLL CLK SELT 64M
    }

}






