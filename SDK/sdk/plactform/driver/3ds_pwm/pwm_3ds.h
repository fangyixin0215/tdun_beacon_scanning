/*************************************************************
 * @file        driver_pwm_3ds.h
 * @brief       Header file of driver_pwm_3ds.c
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par
 * @attention
 *
 * @history     2016-09-29 gwf    create this file
 */

#ifndef __DRIVER_PWM_3DS_H__

#define __DRIVER_PWM_3DS_H__


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


#include <stdint.h>           // standard integer functions
#include "config.h"           // standard integer functions


#define REG_PWM_3DS_BASE_ADDR                   (0x00806800UL)
#define PWM_3DS_CHANNEL_NUMBER_ALL              4
#define PWM_3DS_CHANNEL_NUMBER_MAX              (PWM_3DS_CHANNEL_NUMBER_ALL - 1)
#define PWM_3DS_CHANNEL_NUMBER_MIN              0


#ifdef PWM_3DS_3231_STYLE
#define REG_PWM_3DS_CTRL_ADDR                   (REG_PWM_3DS_BASE_ADDR + 0x00 * 4)
#define REG_PWM_3DS_CTRL_MASK                   0x0000FFFFUL
#define REG_PWM_3DS_CTRL                        (*((volatile unsigned long *) REG_PWM_3DS_CTRL_ADDR))

#define PWM_3DS_CTRL_PWM_ENABLE_POSI            1
#define PWM_3DS_CTRL_PWM_ENABLE_MASK            (0x01UL << PWM_3DS_CTRL_PWM_ENABLE_POSI)

#define PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_POSI    7
#define PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_MASK    (0x01UL << PWM_3DS_CTRL_PWM_3DS_INT_ENABLE_POSI)

#define PWM_3DS_CTRL_PWM_3DS_X_ENABLE_POSI(x)       ((x) + 8)
#define PWM_3DS_CTRL_PWM_3DS_X_ENABLE_MASK(x)       (0x01UL << PWM_3DS_CTRL_PWM_3DS_X_ENABLE_POSI(x))
#define PWM_3DS_CTRL_PWM_3DS_X_ENABLE_SET(x)        (0x01UL << PWM_3DS_CTRL_PWM_3DS_X_ENABLE_POSI(x))

#define PWM_3DS_CTRL_PWM_3DS_X_START_SET_POSI(x)    ((x) + 12)
#define PWM_3DS_CTRL_PWM_3DS_X_START_SET_MASK(x)    (0x01UL << PWM_3DS_CTRL_PWM_3DS_X_START_SET_POSI(x))
#define PWM_3DS_CTRL_PWM_3DS_X_START_SET_SET(x)     (0x01UL << PWM_3DS_CTRL_PWM_3DS_X_START_SET_POSI(x))


#define REG_PWM_3DS_INT_STATUS_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x01 * 4)
#define REG_PWM_3DS_INT_STATUS_MASK             0x0FUL
#define REG_PWM_3DS_INT_STATUS                  (*((volatile unsigned long *) REG_PWM_3DS_INT_STATUS_ADDR))

#define PWM_3DS_INT_STATUS_PWM_POSI             3
#define PWM_3DS_INT_STATUS_PWM_MASK             (0x01UL << PWM_3DS_INT_STATUS_PWM_POSI)

#define REG_PWM_3DS_FRAM_BTCLK_ADDR            (REG_PWM_3DS_BASE_ADDR + 0x10 * 4)
#define REG_PWM_3DS_FRAM_BTCLK                  (*((volatile unsigned long *) REG_PWM_3DS_FRAM_BTCLK_ADDR))


#define REG_PWM_3DS_FRAM_PHASE_ADDR            (REG_PWM_3DS_BASE_ADDR + 0x11 * 4)
#define REG_PWM_3DS_FRAM_PHASE                  (*((volatile unsigned long *) REG_PWM_3DS_FRAM_PHASE_ADDR))

#define REG_PWM_3DS_FRAM_TSYNC_ADDR            (REG_PWM_3DS_BASE_ADDR + 0x12 * 4)
#define REG_PWM_3DS_FRAM_TSYNC                  (*((volatile unsigned long *) REG_PWM_3DS_FRAM_TSYNC_ADDR))


#define REG_PWM_3DS_FRAM_FRAC_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x13 * 4)
#define REG_PWM_3DS_FRAM_FRAC                  (*((volatile unsigned long *) REG_PWM_3DS_FRAM_FRAC_ADDR))


#define REG_PWM_3DS_PWM0_START_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x14 * 4)
#define REG_PWM_3DS_PWM0_START_MASK             0x003FFFFFUL
#define REG_PWM_3DS_PWM0_START                  (*((volatile unsigned long *) REG_PWM_3DS_PWM0_START_ADDR))


#define REG_PWM_3DS_PWM0_MID_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x15 * 4)
#define REG_PWM_3DS_PWM0_MID_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM0_MID                    (*((volatile unsigned long *) REG_PWM_3DS_PWM0_MID_ADDR))


#define REG_PWM_3DS_PWM0_END_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x16 * 4)
#define REG_PWM_3DS_PWM0_END_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM0_END                    (*((volatile unsigned long *) REG_PWM_3DS_PWM0_END_ADDR))


#define REG_PWM_3DS_PWM1_START_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x17 * 4)
#define REG_PWM_3DS_PWM1_START_MASK             0x003FFFFFUL
#define REG_PWM_3DS_PWM1_START                  (*((volatile unsigned long *) REG_PWM_3DS_PWM1_START_ADDR))


#define REG_PWM_3DS_PWM1_MID_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x18 * 4)
#define REG_PWM_3DS_PWM1_MID_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM1_MID                    (*((volatile unsigned long *) REG_PWM_3DS_PWM1_MID_ADDR))


#define REG_PWM_3DS_PWM1_END_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x19 * 4)
#define REG_PWM_3DS_PWM1_END_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM1_END                    (*((volatile unsigned long *) REG_PWM_3DS_PWM1_END_ADDR))


#define REG_PWM_3DS_PWM2_START_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x1A * 4)
#define REG_PWM_3DS_PWM2_START_MASK             0x003FFFFFUL
#define REG_PWM_3DS_PWM2_START                  (*((volatile unsigned long *) REG_PWM_3DS_PWM2_START_ADDR))


#define REG_PWM_3DS_PWM2_MID_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x1B * 4)
#define REG_PWM_3DS_PWM2_MID_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM2_MID                    (*((volatile unsigned long *) REG_PWM_3DS_PWM2_MID_ADDR))


#define REG_PWM_3DS_PWM2_END_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x1C * 4)
#define REG_PWM_3DS_PWM2_END_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM2_END                    (*((volatile unsigned long *) REG_PWM_3DS_PWM2_END_ADDR))


#define REG_PWM_3DS_PWM3_START_ADDR             (REG_PWM_3DS_BASE_ADDR + 0x1D * 4)
#define REG_PWM_3DS_PWM3_START_MASK             0x003FFFFFUL
#define REG_PWM_3DS_PWM3_START                  (*((volatile unsigned long *) REG_PWM_3DS_PWM3_START_ADDR))


#define REG_PWM_3DS_PWM3_MID_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x1E * 4)
#define REG_PWM_3DS_PWM3_MID_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM3_MID                    (*((volatile unsigned long *) REG_PWM_3DS_PWM3_MID_ADDR))


#define REG_PWM_3DS_PWM3_END_ADDR               (REG_PWM_3DS_BASE_ADDR + 0x1F * 4)
#define REG_PWM_3DS_PWM3_END_MASK               0x003FFFFFUL
#define REG_PWM_3DS_PWM3_END                    (*((volatile unsigned long *) REG_PWM_3DS_PWM3_END_ADDR))


#define REG_PWM_3DS_PWM_X_START_ADDR(x)         (REG_PWM_3DS_BASE_ADDR + (0x14 + (x) * 3) * 4)
#define REG_PWM_3DS_PWM_X_START_MASK            0x003FFFFFUL
#define REG_PWM_3DS_PWM_X_START(x)              (*((volatile unsigned long *) REG_PWM_3DS_PWM_X_START_ADDR(x)))


#define REG_PWM_3DS_PWM_X_MID_ADDR(x)           (REG_PWM_3DS_BASE_ADDR + (0x15 + (x) * 3) * 4)
#define REG_PWM_3DS_PWM_X_MID_MASK              0x003FFFFFUL
#define REG_PWM_3DS_PWM_X_MID(x)                (*((volatile unsigned long *) REG_PWM_3DS_PWM_X_MID_ADDR(x)))


#define REG_PWM_3DS_PWM_X_END_ADDR(x)           (REG_PWM_3DS_BASE_ADDR + (0x16 + (x) * 3) * 4)
#define REG_PWM_3DS_PWM_X_END_MASK              0x003FFFFFUL
#define REG_PWM_3DS_PWM_X_END(x)                (*((volatile unsigned long *) REG_PWM_3DS_PWM_X_END_ADDR(x)))
#endif      /* #ifdef PWM_3231_STYLE */




//----------------------------------------------
// PWM_3DS driver description
//----------------------------------------------
typedef struct
{
    unsigned long       start_value;    // PWM_3DS counter start value
    unsigned long       end_value;      // PWM_3DS counter end value
    unsigned long       duty_cycle;     // PWM_3DS counter duty cycle, this value must smaller or equal to end value
    unsigned char       channel;        // PWM_3DS 0~3, GPIOC P2.0~P2.3
} PWM_3DS_DRV_DESC;


extern STATUS PWM_3DS_init(PWM_3DS_DRV_DESC *p_PWM_3DS_drv_desc);
extern void PWM_3DS_enable(unsigned char ucChannel);
extern void PWM_3DS_disable(unsigned char ucChannel);
extern void PWM_3DS_int_enable(unsigned char ucChannel);
extern void PWM_3DS_int_disable(unsigned char ucChannel);
extern void PWM_3DS_setduty(PWM_3DS_DRV_DESC *p_PWM_3DS_drv_desc);
extern void PWM_3DS_int(unsigned char ucChannel);
extern void PWM_3DS_Test(unsigned char ucChannel, unsigned long ulStartValue,
				   unsigned long ulDutyCycle, unsigned long ulEndValue);


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __DRIVER_PWM_3DS_H__ */
