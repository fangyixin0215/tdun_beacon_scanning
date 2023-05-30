/*************************************************************
 * @file        driver_pwm_3ds.c
 * @brief       code of PWM_3DS driver of BK3435
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par
 * @attention
 *
 * @history     2016-09-29 gwf    create this file
 */

#include <stdio.h>

#include "icu.h"
#include "gpio.h"

#include "pwm_3ds.h"
#include "driver_icu.h"

#define UART_PRINTF	uart_printf
int uart_printf(const char *fmt,...);

extern void DelayMS(volatile unsigned long timesMS);


STATUS PWM_3DS_init(PWM_3DS_DRV_DESC *p_PWM_3DS_drv_desc)
{
    if (p_PWM_3DS_drv_desc == NULL)
    {
        return ERROR;
    }

    if (p_PWM_3DS_drv_desc->channel > PWM_3DS_CHANNEL_NUMBER_MAX)
    {
        return ERROR;
    }

    if (p_PWM_3DS_drv_desc->duty_cycle > p_PWM_3DS_drv_desc->end_value)
    {
        return ERROR;
    }

    // enable GPIO second function
    REG_APB5_GPIOC_CFG = 0x0000ff00;//~(0x1<<(p_PWM_3DS_drv_desc->channel));
    REG_APB5_GPIOC_DATA = 0;

    // set PWM_3DS clock enable and clock source
    REG_ICU_BLE_CLK &= (~ICU_BLE_CLK_PWD_MASK);
    REG_ICU_PWM_CLK &= (~ICU_PWM_CLK_PWM_3DS_PWD_MASK);

    // set end_value and duty_cycle
    REG_PWM_3DS_FRAM_TSYNC	= ((p_PWM_3DS_drv_desc->end_value-1)>>4);
    REG_PWM_3DS_FRAM_FRAC	= ((p_PWM_3DS_drv_desc->end_value-1)&0x0f)<<4;
    REG_PWM_3DS_FRAM_BTCLK=0;

    REG_PWM_3DS_PWM_X_START(p_PWM_3DS_drv_desc->channel) = p_PWM_3DS_drv_desc->start_value;
    REG_PWM_3DS_PWM_X_MID(p_PWM_3DS_drv_desc->channel)   = p_PWM_3DS_drv_desc->duty_cycle;
    REG_PWM_3DS_PWM_X_END(p_PWM_3DS_drv_desc->channel)   = p_PWM_3DS_drv_desc->end_value;

    // set PWM_3DS enable, PWM_3DS int enable
    REG_PWM_3DS_CTRL = PWM_3DS_CTRL_PWM_3DS_X_ENABLE_MASK(p_PWM_3DS_drv_desc->channel)
                     | PWM_3DS_CTRL_PWM_3DS_X_START_SET_MASK(p_PWM_3DS_drv_desc->channel)
                     | PWM_3DS_CTRL_PWM_ENABLE_MASK;
//                     | (((p_PWM_3DS_drv_desc->mode >> 1) & 0x01) << PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_POSI);
  //  UART_PRINTF("3d=%x,%x,%x,%x\r\n", p_PWM_3DS_drv_desc->start_value, p_PWM_3DS_drv_desc->duty_cycle, p_PWM_3DS_drv_desc->end_value,p_PWM_3DS_drv_desc->channel);

//    UART_PRINTF("3d=%x,%x,%x,%x\r\n",REG_PWM_3DS_PWM2_START,REG_PWM_3DS_PWM2_MID,REG_PWM_3DS_PWM2_END,REG_PWM_3DS_CTRL);

    return OK;
}

void PWM_3DS_setduty(PWM_3DS_DRV_DESC *p_PWM_3DS_drv_desc)
{

   REG_PWM_3DS_PWM_X_MID(p_PWM_3DS_drv_desc->channel)   = p_PWM_3DS_drv_desc->duty_cycle;

}

void PWM_3DS_enable(unsigned char ucChannel)
{
    if (ucChannel > PWM_3DS_CHANNEL_NUMBER_MAX)
    {
        return;
    }
    REG_PWM_3DS_CTRL |= ( PWM_3DS_CTRL_PWM_3DS_X_ENABLE_MASK(ucChannel));
}

void PWM_3DS_disable(unsigned char ucChannel)
{
    if (ucChannel > PWM_3DS_CHANNEL_NUMBER_MAX)
    {
        return;
    }
    REG_PWM_3DS_CTRL &= (~PWM_3DS_CTRL_PWM_3DS_X_ENABLE_MASK(ucChannel));
}

void PWM_3DS_int_enable(unsigned char ucChannel)
{
    if (ucChannel > PWM_3DS_CHANNEL_NUMBER_MAX)
    {
        return;
    }
    REG_PWM_3DS_CTRL |= PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_MASK;
}

void PWM_3DS_int_disable(unsigned char ucChannel)
{
    if (ucChannel > PWM_3DS_CHANNEL_NUMBER_MAX)
    {
        return;
    }
    REG_PWM_3DS_CTRL &= (~PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_MASK);
}


void PWM_3DS_int(unsigned char ucChannel)
{
    unsigned long ulIntStatus;

    ulIntStatus = REG_PWM_3DS_INT_STATUS;

    REG_PWM_3DS_INT_STATUS = ulIntStatus;
}

/*************************************************************
 * PWM_3DS_Test1
 * Description: PWM_3DS output test
 * Parameters:  none
 * return:      none
 * error:       none
 */
void PWM_3DS_Test(unsigned char ucChannel, unsigned long ulStartValue,
				   unsigned long ulDutyCycle, unsigned long ulEndValue)
{
    PWM_3DS_DRV_DESC pwm_3ds_drv_desc;

    UART_PRINTF("----- PWM_3DS_Test start -----\r\n");
    ulDutyCycle = 0;

    pwm_3ds_drv_desc.channel = ucChannel;                   // PWM_3DS channel
    pwm_3ds_drv_desc.start_value= 0;                        // PWM_3DS start_value
    pwm_3ds_drv_desc.duty_cycle = ulDutyCycle;              // PWM_3DS duty_cycle
    pwm_3ds_drv_desc.end_value  = ulEndValue;               // PWM_3DS end_value
    PWM_3DS_init(&pwm_3ds_drv_desc);
    while(1)
    {
        DelayMS(10);
        pwm_3ds_drv_desc.duty_cycle++;
        if( pwm_3ds_drv_desc.end_value  == pwm_3ds_drv_desc.duty_cycle)
        {
            pwm_3ds_drv_desc.duty_cycle = 0;
        }
        PWM_3DS_setduty(&pwm_3ds_drv_desc);
    }
    PWM_3DS_disable(ucChannel);

    UART_PRINTF("----- PWM_3DS_Test over  -----\r\n");
}

