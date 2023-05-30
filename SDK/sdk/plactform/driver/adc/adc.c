#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stddef.h>     // standard definition
#include "BK3435_reg.h"
#include "uart.h"
#include "gpio.h"
#include "adc.h"


uint16_t g_adc_value,adc_flag,referance_voltage;

/************************************************************************
//ADC参考电压默认为1.2V
//确保ADC稳定采样，ADC口需要接个10nf到地的电容
*************************************************************************/
void adc_init(uint8_t chanle,uint8_t mode)
{
	uint32_t cfg;

	//enable adc clk
	REG_AHB0_ICU_ADCCLKCON &= ~(0x01 << 0);
	//adc div
	REG_AHB0_ICU_ADCCLKCON = (0x5 << 1);
		
	//set special as peripheral func
	gpio_config(GPIOD_0 + chanle,FLOAT,PULL_NONE); 

	//set adc mode/channel/wait clk
	cfg  = ( (mode << BIT_ADC_MODE ) 
	         | (chanle << BIT_ADC_CHNL) 
	         | (0x01 << BIT_ADC_WAIT_CLK_SETTING));
	
	//set adc sample rate/pre div
	cfg |= ((18 << BIT_ADC_SAMPLE_RATE) 
	        | (3 << BIT_ADC_PRE_DIV)
	        |(0x0 << BIT_ADC_DIV1_MODE)
	        |(0x0 << BIT_ADC_FILTER)
	        |(0x01 << BIT_ADC_INT_CLEAR));

	REG_APB7_ADC_CFG =  cfg;
	
	REG_AHB0_ICU_INT_ENABLE |= (0x01 << 8); 
    //REG_APB7_ADC_CFG |= (0x01 << BIT_ADC_EN);////非连续模式不能先使能ADC，不然ADC FIFO满时没有读出再次启动ADC就不会有中断
    if(mode==3)
        REG_APB7_ADC_CFG |= (0x01 << BIT_ADC_EN);
        
}

void adc_deinit(uint8_t channel)
{
    gpio_config(0x30 + channel,INPUT,PULL_HIGH);

    REG_APB7_ADC_CFG &= ~(SET_ADC_EN+(0x03 << BIT_ADC_MODE ));
    REG_AHB0_ICU_INT_ENABLE &= ~(0x01 << 8); ;
    REG_AHB0_ICU_ADCCLKCON |= (0x01 << 0);;
}
void adc_isr(void)
{
	REG_APB7_ADC_CFG |= (0x01 << BIT_ADC_INT_CLEAR);
	
    adc_flag=1;	
	if((REG_APB7_ADC_CFG&0x03)==0x03)
		uart_printf("adc_value=%x\r\n",REG_APB7_ADC_DAT);
}

extern void Delay_us(int num);

uint16_t adc_get_value(uint8_t channel,uint8_t mode)
{
    uint16_t adc_cnt;
    uint16_t adc_data_max,adc_data_min;
    uint32_t adc_data_sum;
    uint8_t  i=0;

    adc_cnt=0;
    adc_flag =0;

    if((REG_APB7_ADC_CFG&0x03)==0x03)
        return 0;
    
    REG_APB7_ADC_CFG |= SET_ADC_EN+(mode << BIT_ADC_MODE )+(channel << BIT_ADC_CHNL);
    
    while (!adc_flag)  
    {
        adc_cnt++;       
        if(adc_cnt>300)
        {
            UART_PRINTF("g_adc_value_timeout\r\n");
            break;			
        }
        Delay_us(10);
    } 
    if(adc_flag==1)
    {
        if(mode==1)
        {
            g_adc_value=REG_APB7_ADC_DAT>>2;
            UART_PRINTF("g_adc_value=%x\r\n",g_adc_value);
        }
        else if(mode==2)
        {
            adc_data_sum = 0;
            adc_data_max= 0;
            adc_data_min = 0x3ff;
            g_adc_value=REG_APB7_ADC_DAT>>2; // don't use
            
            for(i=0;i<6;i++) 
            {
                while(REG_APB7_ADC_CFG&(0x01<<BIT_ADC_FIFO_EMPTY))
                {
                    adc_cnt++;       
                    if(adc_cnt>300)
                    {
                        UART_PRINTF("g_adc_value_timeout2\r\n");
                        break;			
                    }
                }
                g_adc_value=REG_APB7_ADC_DAT>>2;
                
                adc_data_sum += g_adc_value;
                if(adc_data_max < g_adc_value)
                adc_data_max = g_adc_value;
                if(adc_data_min > g_adc_value)
                adc_data_min = g_adc_value;
                
            }

            adc_data_sum = adc_data_sum - adc_data_max - adc_data_min;

            g_adc_value = adc_data_sum>>2;
            UART_PRINTF("g_adc_value2=%x,\r\n",g_adc_value);

        }
    }
    
    REG_APB7_ADC_CFG &= ~(SET_ADC_EN+(0x03 << BIT_ADC_MODE )+(0x0f << BIT_ADC_CHNL)); //ADC值读取完成后必须把使能位清除     
    
    return g_adc_value;     
}

/**************************************************************************
//ADC校准
//校准ADC需要给芯片一个稳定的供电压，然后算ADC参考电压
//这个函数校准默认使用3V电源供电,内部分压后为0.75V
*************************************************************************/
#define CALIB_COUNT 6
#define STABL_VALT 75///分压后的0.75V稳定电压

void calib_adc(void)
{
#if(ADC_DRIVER)    
    uint8_t i;
    static uint16_t calib_temp=0;
    adc_init(8,2);

    for(i=0;i<CALIB_COUNT;i++)
    {
        calib_temp += adc_get_value(8,2);
        Delay_us(1000);
    }
    
    referance_voltage=(0xff*STABL_VALT)/(calib_temp/CALIB_COUNT);///计算后值为121，那么参考电压就为1.21V
    UART_PRINTF("referance_voltage=%d\r\n",referance_voltage);
#endif    
}

