/*************************************************************
 * @file        driver_i2s.c
 * @brief       code of I2S driver of BK3435
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par
 * @attention
 *
 * @history     2016-09-29 gwf    create this file
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include "icu.h"
#include "gpio.h"
#include "i2s.h"
#include "BK3435_reg.h"
#include "driver_icu.h"

#define UART_PRINTF	uart_printf
int uart_printf(const char *fmt,...);

#if 1
#define I2S_SYS_CLK                     PER_CLK

volatile i2s_message		i2s_msg;
//i2s_message		i2s_msg;
unsigned int	*p_i2s_tx0_data;
//unsigned int	*p_i2s_tx1_data;
//unsigned int	*p_i2s_tx2_data;
unsigned int	*p_i2s_rx_data;

#define I2S_DATA_SIZE			150
unsigned int	i2s_tx_data[I2S_DATA_SIZE], i2s_rx_data[I2S_DATA_SIZE];


#define GPIO_I2S_CFG            	REG_APB5_GPIOC_CFG
#define GPIO_I2S_1_CFG            	REG_APB5_GPIOD_CFG

#define GPIO_I2S_BCLK_PIN                   5
#define GPIO_I2S_SCLK_PIN                   6
#define GPIO_I2S_DIN_PIN                    7
#define GPIO_I2S_DOUT_PIN                   0

#define NUMBER_ROUND_UP(a,b)        ((a) / (b) + (((a) % (b)) ? 1 : 0))

extern volatile uint32_t XVR_ANALOG_REG_BAK[16] ;
void DelayNops(volatile unsigned long nops)
{
    while (nops --)
    {
    }
}

void DelayMS(volatile unsigned long timesMS)
{
	volatile unsigned long i;

#if (MCU_CLK == MCU_CLK_32KHz)
	#define 	tick_cnt_ms 		8
#elif (MCU_CLK == MCU_CLK_16MHz)
	#define 	tick_cnt_ms 		1660
#elif (MCU_CLK == MCU_CLK_48MHz)
	#define 	tick_cnt_ms 		3000
#else
	#define 	tick_cnt_ms 		(1660 * 3)
#endif
	while (timesMS --)
	{
		i = 0;
		while (i < tick_cnt_ms)
		{
			i++;
		}
	}
}


void i2s_init(unsigned long smp_ratio, unsigned char role, unsigned char data_width,
			  unsigned char mode_sel, unsigned long data_cnt)
{
	unsigned long bitclk_div;
//	unsigned long smp_dly_cfg;
	unsigned long lrck_div;

    ICU_I2S_PWD_CLEAR();

    // fix analog BUG
    {
        volatile unsigned long *p_APB_XVER_REG;
        p_APB_XVER_REG = (volatile unsigned long *)APB_XVER_BASE;
        p_APB_XVER_REG[0x0E] = XVR_ANALOG_REG_BAK[0x0E] = XVR_ANALOG_REG_BAK[0x0E] | (1UL<<29) | (1UL<<31);
        REG_ICU_ANALOG_TEST |=   (1<<18);
        REG_ICU_ANALOG_PWD  &= (~(1<<18));
    }
    // Enable GPIO  peripheral function for i2s
    GPIO_I2S_CFG &= ~((0x1<<GPIO_I2S_BCLK_PIN)  | (0x1<<GPIO_I2S_SCLK_PIN)
                       | (0x1<<GPIO_I2S_DIN_PIN) );
    GPIO_I2S_1_CFG &= ~(0x1<<GPIO_I2S_DOUT_PIN );


    if (smp_ratio != 8000 && smp_ratio != 16000 && smp_ratio != 44100 && smp_ratio != 48000)
    {
        return;
    }
/*
    if (data_width > 32)
    {
        return;
    }*/

	// pcm clock frequency is 26MHz
/*	if (smp_ratio)	//16KHz
	{
		bitclk_div = 25;
		smp_dly_cfg = 25;
		lrck_div = 15;
	}
	else //8K
	{
		bitclk_div = 50;
		smp_dly_cfg = 50;
		lrck_div = 31;
	}*/
    if (data_width < 15)
    {
        lrck_div = 15;
    }
    else if (data_width < 19)
    {
        lrck_div = 19;
    }
    else if (data_width < 23)
    {
        lrck_div = 23;
    }
    else
    {
        lrck_div = 31;
    }

    // bitclk_div <= 4 的时候接收数据会出错，所以这个值低于5是不能用的
    bitclk_div = NUMBER_ROUND_UP((I2S_SYS_CLK / 2 ), (smp_ratio * 2 * (lrck_div + 1))) - 1;
    UART_PRINTF("bitclk_div = 0x%lx\r\n", bitclk_div);

	REG_I2S_CONTRL = ((bitclk_div & 0x00FF) << 0)
                   | ( lrck_div             << 8)
                   | ( data_width           << 16)
                   | ((role & 0x01)         << 30)
                   | ((mode_sel & 0x07)     << 27);
	REG_I2S_CONFIG = 0x000005ff; //| ((bitclk_div & 0x0000ff00) << 16) | (smp_dly_cfg << 16);
    REG_I2S_STATUS = REG_I2S_STATUS_MASK;

    UART_PRINTF("REG_I2S_CONTRL = 0x%lx, REG_I2S_CONFIG = 0x%lx, REG_I2S_STATUS = 0x%lx\r\n",
            REG_I2S_CONTRL, REG_I2S_CONFIG, REG_I2S_STATUS);

	i2s_msg.rxfifo_level = 1;
	i2s_msg.txfifo0_level = 1;
	i2s_msg.remain_data_cnt = data_cnt;
	i2s_msg.trans_done = 0;
}

void i2s_disable(void)
{
	REG_I2S_CONTRL &= (~I2S_CONTRL_I2S_PCM_ENABLE_MASK);
    ICU_I2S_PWD_SET();
    ICU_INT_ENABLE_CLEAR(ICU_INT_ENABLE_IRQ_I2S_MASK);
}

void i2s_enable(void)
{
	REG_I2S_CONTRL |= I2S_CONTRL_I2S_PCM_ENABLE_MASK;
    ICU_I2S_PWD_CLEAR();
    ICU_INT_ENABLE_SET(ICU_INT_ENABLE_IRQ_I2S_MASK);
}

void i2s_clrfifo(unsigned char fifo_idx)
{
	//[0]:rxfifo, [1]:txfifo0, [2]:txfifo1, [3]:txfifo2
 	REG_I2S_CONFIG |= fifo_idx << 16;
}

void i2s_isr(void)
{
	unsigned long		i2s_stat;
	unsigned char		rxint, txint0;
	unsigned int		i;
	volatile unsigned int data_num;

	i2s_stat = REG_I2S_STATUS;
//	rxint  = i2s_stat & (0x01 | 0x10);
//	txint0 = i2s_stat & (0x02 | 0x20);
	rxint  = i2s_stat & 0x01;
	txint0 = i2s_stat & 0x02;
	//txint1 = i2s_stat & 0x04;
	//txint2 = i2s_stat & 0x08;

//    printf("REG_I2S_STATUS = 0x%lx\r\n", i2s_stat);
	if (txint0)
	{
		switch (i2s_msg.txfifo0_level)
		{
			case 0	: 	data_num = 24;  break;
			case 1	: 	data_num = 16;  break;
			case 2	: 	data_num = 8;   break;
			default	:  	data_num = 32;  break;
		}

		for (i=0; i<data_num; i++)
		{
			REG_I2S_DATA0 = *p_i2s_tx0_data;
			p_i2s_tx0_data ++;
		}
		i2s_stat |= 0x2;
	}

	if (rxint)
	{
		switch (i2s_msg.rxfifo_level)
		{
			case 0	: 	data_num = 24;  break;
			case 1	: 	data_num = 16;  break;
			case 2	: 	data_num = 8;   break;
			default	:  	data_num = 32;  break;
		}
        if (data_num > i2s_msg.remain_data_cnt)
        {
            data_num = i2s_msg.remain_data_cnt;
        }
		for (i=0; i<data_num; i++)
		{
			*p_i2s_rx_data = REG_I2S_DATA0;
			p_i2s_rx_data ++;
		}
		i2s_msg.remain_data_cnt -= data_num;
		if (i2s_msg.remain_data_cnt <= 0)
		{
			i2s_msg.trans_done = 1;
		}
		i2s_stat |= 0x1;
	}

    REG_I2S_STATUS = i2s_stat;
}

void I2S_Test(unsigned char role)	//0: Slave; 1: Master
{
	int             i, j;
	unsigned char	data_width;//, role;
	unsigned long	/*temp,*/ err_cnt, total_err_cnt;
	unsigned char   mode_sel_cnt, smp_ratio_cnt;
    unsigned char   mode_sel[7] = {0, 1, 2, 4, 5, 6, 7};
    unsigned long   smp_ratio[4] = {8000, 16000, 44100, 48000};
    unsigned int    i2s_rx_expected_data[I2S_DATA_SIZE];

    UART_PRINTF("----- I2S_Test %s start -----\r\n", (role) ? ("Master") : ("Slave"));

//	err_cnt = 0;
	total_err_cnt = 0;

//	role = 0;	//0: Slave; 1: Master
    if (role)
    {
        for (i=0; i<TEST_DATA_LEN+8; i++)
        {
            i2s_tx_data[i] = ((i+1)<<24) | ((i+1)<<16)  | ((i+1)<<8) | ((i+1)<<0) | 0x80808080;
        }
        for (i=0; i<TEST_DATA_LEN+8; i++)
        {
        	i2s_rx_expected_data[i] = ((i+1)<<24) | ((i+1)<<16)  | ((i+1)<<8) | ((i+1)<<0) | 0xC0C0C0C0;
        }
        UART_PRINTF("Master:\r\n");
    }
    else
    {
        for (i=0; i<TEST_DATA_LEN+8; i++)
        {
            i2s_tx_data[i] = ((i+1)<<24) | ((i+1)<<16)  | ((i+1)<<8) | ((i+1)<<0) | 0xC0C0C0C0;
        }
        for (i=0; i<TEST_DATA_LEN+8; i++)
        {
        	i2s_rx_expected_data[i] = ((i+1)<<24) | ((i+1)<<16)  | ((i+1)<<8) | ((i+1)<<0) | 0x80808080;
        }
        UART_PRINTF("Slave:\r\n");
    }

	//if(!role) PERI_PWDS &= ~bit_PCM2_CLK_SEL;     //select xtal as pcm clock in slave mode

	for (smp_ratio_cnt=0; smp_ratio_cnt<4; smp_ratio_cnt++)
	{
//		if (smp_ratio == 8000)		// 8KHz
// 		{
// 			mode_sel_start = 0;
// 			mode_sel_end = 3;
// 			data_width = 15;
// 		}
// 		else	// 8KHz
// 		{
// 			mode_sel_start = 4;
// 			mode_sel_end = 6;
// 			data_width = 15;
// 		}

// 		data_width = 15;
		for (data_width=15; data_width<31; data_width+=4)
		{
			for (mode_sel_cnt=0; mode_sel_cnt<7; mode_sel_cnt++)
//             mode_sel = 0;
//             data_width = 15;
			{
				//printf("DataWidth = %d, ModeSel = %d!\r\n",data_width, mode_sel);

                UART_PRINTF("\r\nsmp_ratio_cnt=%d, role=%d, data_width=%d, mode_sel_cnt=%d\r\n",
                        smp_ratio_cnt, role, data_width, mode_sel_cnt);
				i2s_init(smp_ratio[smp_ratio_cnt], role, data_width, mode_sel[mode_sel_cnt], TEST_DATA_LEN);

// 				for (j = 0; j < 150; j ++)
// 				{
// 					temp = 65536 - (j + 1) * (smp_ratio + 1) * data_width * 100;
// 					switch (data_width)
// 					{
// 						case 7 	: 	temp_data = temp & 0xff;	    break;
// 						case 15	:	temp_data = temp & 0xffff;      break;
// 						case 23	:	temp_data = temp & 0xffffff;    break;
// 						default	:	temp_data = temp;	            break;
// 					}
// 					i2s_tx_data[j] = temp_data;
// 				}
//                if (role)
//                {
//                    memset(i2s_rx_data, 0, sizeof(i2s_rx_data));
//                }
//                else
                {
                    memset(i2s_rx_data, 0, sizeof(i2s_rx_data));
                }

				p_i2s_tx0_data = i2s_tx_data + mode_sel_cnt;
				//p_i2s_tx1_data = i2s_tx_data;
				//p_i2s_tx2_data = i2s_tx_data;
				p_i2s_rx_data = i2s_rx_data;

				if (role)
                {
                    DelayMS(3000);
                }
				else
                {
                    DelayMS(1000);
                }
				i2s_enable();
				//gpio_sendout(0xcd);

				while (!i2s_msg.trans_done)
				{
					//dbg_cnt ++;
				}
				i2s_msg.trans_done = 0;

				//gpio_sendout(0xce);

				i2s_disable();
				i2s_clrfifo(15);

				//compare Rx Data
#if 0
				if (role)
				{
					err_cnt = 0;
					i = 0;
					j = 0;
					while (!i2s_rx_data[i])
                    {
                        i++;
                    }
					while (i < TEST_DATA_LEN)
					{
						if (i2s_rx_data[i] != i2s_tx_data[j])
						{
							//UART_PRINTF("I2S Data Error! should be(%02X) but(%02X)!\r\n", i2s_tx_data[i], i2s_rx_data[i]);
							err_cnt ++;
						}
						//else
							//UART_PRINTF("I2S Data OK! should be(%02X) actual be(%02X)!\r\n", i2s_tx_data[i], i2s_rx_data[i]);
						i ++;
						j ++;
					}
					//UART_PRINTF("I2S Error Counter = %d!\r\n", err_cnt);
//					gpio_sendout(0xb0);
//					gpio_sendout(err_cnt);  //output compare result
					DelayNops(200);
//					gpio_sendout(0xb1);
					total_err_cnt += err_cnt;
				}
                UART_PRINTF("I2S Error Counter = %d\r\n", err_cnt);
#else
				do
				{
					unsigned long ulTemp;
					ulTemp = 0;
					err_cnt = 0;
					for (i=0; i<=data_width; i++)
					{
						ulTemp |= (0x01UL << i);
					}
					UART_PRINTF("ulTemp = 0x%x\r\n", ulTemp);

                    for (i=0; i<TEST_DATA_LEN; i++)
                    {
                        if (i2s_rx_data[i] != 0)
                        {
                        	break;
                        }
                    }
                    if (i == TEST_DATA_LEN)
                    {
                    	UART_PRINTF("i2s_rx_data search failed!\r\n");
                    	break;		// 没收到数据
                    }

                    for (j=0; j<TEST_DATA_LEN; j++)
                    {
                        if (i2s_rx_data[i] == (i2s_rx_expected_data[j] & ulTemp))
                        {
                        	break;
                        }
                    }
                    if (j == TEST_DATA_LEN)
                    {
                    	UART_PRINTF("i2s_rx_expected_data search failed!\r\n");
                    	break;		// 没找到与第一个接收到的数据对应的i2s_rx_expected_data的数据
                    }
                    j = j-i;
					UART_PRINTF("i = %d, j = %d\r\n", i, j);

					if (role)
					{
						for (i=0; i<TEST_DATA_LEN; i++)
						{
							if (i2s_rx_data[i] != (i2s_rx_expected_data[i+j] & ulTemp))
							{
								*((volatile unsigned long *) 0x00F50000UL + 0x00 * 4) = i2s_rx_data[i];
								UART_PRINTF("i2s_rx_data[%d]=0x%x, i2s_rx_expected_data[%d]=0x%x\r\n",
										i, i2s_rx_data[i], i+j, i2s_rx_expected_data[i+j] & ulTemp);
								err_cnt ++;
							}
						}
					}
					else
					{
						*((volatile unsigned long *) 0x00F50000UL + 0x01 * 4) = i2s_tx_data[i];
					}
					total_err_cnt += err_cnt;
	                UART_PRINTF("I2S Error Counter = %d\r\n", err_cnt);
				}
				while (0);
#endif
                for (i=0; i<TEST_DATA_LEN; i++)
                {
                    UART_PRINTF("i2s_rx_data[%d]=0x%x, i2s_tx_data[%d]=0x%x\r\n", i, i2s_rx_data[i], i, i2s_tx_data[i]);
                }
				DelayNops(200);
			}
		}
	}

//	if (role)
	{
        UART_PRINTF("total_err_cnt = %d\r\n", total_err_cnt);
//		gpio_sendout(0xe0);
//		gpio_sendout(total_err_cnt);  //output compare result
//		DelayNops(200);
//		gpio_sendout(0xa5);
	}

    UART_PRINTF("----- I2S_Test %s over  -----\r\n", (role) ? ("Master") : ("Slave"));
}


#else

#define I2S_MST_MASTER                  1
#define I2S_MST_SLAVE                   0

#define I2S_SYS_CLK                     MCU_CLK_26MHz

#define AUDIO_EMPTY_COUNT_THRE  20

void pcm_fill_buffer(uint8 *buff, uint16 len)
{
}
int rb_get_buffer_with_length(driver_ringbuff_t *rb, uint8 *buff, uint16 len)
{
    return 0;
}

enum
{
    I2S_MODE_I2S = 0,
    I2S_MODE_LEFT_JUST = 1,
    I2S_MODE_RIGHT_JUST = 2,
    I2S_MODE_SHORT_SYNC = 4,
    I2S_MODE_LONG_SYNC = 5,
    I2S_MODE_NORMAL_2B_D = 6,
    I2S_MODE_DELAY_2B_D = 7
};


void i2s_init(unsigned char smp_ratio, unsigned char role, unsigned char data_width, unsigned char mode_sel)
{
    ICU_PERI_CLK_PWD_CLEAR(ICU_PERI_CLK_PWD_I2S_PCM_MASK);
//    ICU_PERI_CLK_GATE_DIS_CLEAR(ICU_PERI_CLK_GATE_DIS_I2S_PCM_MASK);
    GPIO_I2S_function_enable(0);

    REG_I2S_CONFIG = 0;//I2S_CONFIG_TX_FIFO0_LEVEL_SET_8 | I2S_CONFIG_RX_FIFO_LEVEL_SET_16;

//     BK3000_I2S_MODE = ( 2 << sft_I2S_TXFIFO_CTL_TXFIFO_MODE )
//             | ( 5 << sft_I2S_TXFIFO_CTL_DOWN_SMPRATIO );

//    codec_init();
}

void i2s_cfg(uint32 freq, uint16 datawidth)
{
    uint32 bitratio, smpratio;

    if (freq != 8000 && freq != 16000 && freq != 44100 && freq != 48000)
    {
        return;
    }

    if (datawidth > 32)
    {
        return;
    }

    smpratio = 31;
    bitratio = NUMBER_ROUND_UP((I2S_SYS_CLK / 2 ), (freq * 2 * (smpratio + 1))) - 1;
    printf("bitratio = 0x%x\r\n", bitratio);

    REG_I2S_CONTRL = ( bitratio       << I2S_CONTRL_BIT_RATE_POSI)
                   | ( smpratio       << I2S_CONTRL_SAMPLE_RATE_POSI)
                   | ( 0              << I2S_CONTRL_PCM_DLEN_POSI)
                   | ((datawidth - 1) << I2S_CONTRL_DATA_LEN_POSI)
                   | ( 0              << I2S_CONTRL_SYNC_LEN_POSI)
                   | ( 0              << I2S_CONTRL_LSB_FIRST_POSI)
                   | ( 0              << I2S_CONTRL_SCLK_INV_POSI)
                   | ( 0              << I2S_CONTRL_LRCKRP_POSI)
                   | ( I2S_MODE_I2S   << I2S_CONTRL_MODE_SEL_POSI)
                   | ( I2S_MST_MASTER << I2S_CONTRL_MASTER_ENABLE_POSI);

//    codec_cfg(freq, datawidth);
//    i2s_volume_mute(0);
}

void i2s_open(AUDIO_CTRL_BLK *ablk)
{
    REG_I2S_CONTRL |=  I2S_CONTRL_I2S_PCM_ENABLE_SET;
    REG_I2S_CONFIG = 0x11;//(I2S_CONFIG_TX_INT0_ENABLE_SET | I2S_CONFIG_TX_UDF0_ENABLE_SET);
    REG_I2S_STATUS = REG_I2S_STATUS_MASK;
    printf("REG_I2S_CONTRL = 0x%x, REG_I2S_CONFIG = 0x%x, REG_I2S_STATUS = 0x%x\r\n",
            REG_I2S_CONTRL, REG_I2S_CONFIG, REG_I2S_STATUS);

    ICU_INT_ENABLE_SET(ICU_INT_ENABLE_IRQ_I2S_PCM_MASK);
//    printf("i2s_open\r\n");
}

void i2s_close(void)
{
    REG_I2S_CONFIG = 0;
    REG_I2S_CONTRL &= (~I2S_CONTRL_I2S_PCM_ENABLE_SET);

    ICU_INT_ENABLE_CLEAR(ICU_INT_ENABLE_IRQ_I2S_PCM_MASK);
}

void I2S_InterruptHandler(void)
{
    uint32 status;
    uint16 sample[32];
    uint16 len = 0;
    uint8 empty_flag = 0;
    int i;
    AUDIO_CTRL_BLK *ablk = NULL;

//    printf("in I2S_InterruptHandler\r\n");
    status = REG_I2S_STATUS;
    printf("REG_I2S_STATUS = 0x%x\r\n", status);
    if (ablk && (status & (I2S_STATUS_TX_INT0_FLAG_MASK | I2S_STATUS_TX_UDF0_FLAG_MASK)))
    {
        if (ablk->channels == 2)
        {
            len = rb_get_buffer_with_length(&ablk->aud_rb, (uint8 *)sample, 32*2);

            if (len == 0)
            {
                empty_flag = 1;
            }
            else
            {
                for (i = 0; i < len/2; i+= 2)
                {
                    REG_I2S_DATA0 = sample[i];
                    REG_I2S_DATA0 = sample[i+1];
                }
            }
        }
        else
        {
            len = rb_get_buffer_with_length(&ablk->aud_rb, (uint8 *)sample, 4*2);

            if (len == 0)
            {
                empty_flag = 1;
            }
            else
            {
                int j;
                for (i = 0; i < len/2; i++)
                {
                    for (j = 0; j < 12; j++)
                    {
                        REG_I2S_DATA0 = sample[i];
                    }
                }
            }
        }
    }

    if (status & (I2S_STATUS_RX_INT_FLAG_MASK | I2S_STATUS_RX_OVF_FLAG_MASK))
    {
        uint16 dummy;

        for (i = 0; i < 25; i++)
        {
            sample[i] = REG_I2S_DATA0;
            dummy = REG_I2S_DATA0;
            printf("sample[%d] = 0x%x\r\n", i, sample[i]);
            printf("dummy = 0x%x\r\n", dummy);
        }

// #ifdef CONFIG_APP_AEC
//         if (app_bt_is_flag_set(APP_FLAG_HFP_CALLSETUP))
//             app_aec_fill_txbuff( (uint8 *)&sample[0], 16*2 );
//         else
//             pcm_fill_buffer( (uint8 *)&sample[0], 16*2 );
// #else
//         pcm_fill_buffer( (uint8 *)&sample[0], 16*2);
// #endif
    }

    if (ablk)
    {
        if ((status & I2S_STATUS_TX_UDF0_FLAG_MASK) || empty_flag)
        {
            ablk->empty_count ++;
            if (ablk->empty_count >= AUDIO_EMPTY_COUNT_THRE)
            {
                REG_I2S_CONFIG &= (~(I2S_CONFIG_TX_INT0_ENABLE_MASK | I2S_CONFIG_TX_UDF0_ENABLE_MASK));
                ablk->empty_count = 0;
#ifdef DEBUG_BEKEN
                ablk->aud_empty_count++;
#endif
            }
        }
        else
        {
            ablk->empty_count = 0;
        }
    }

    REG_I2S_STATUS = status;
}

void i2s_rx_enable(uint8 enable)
{
    if (enable)
    {
        REG_I2S_CONFIG |= (I2S_CONFIG_RX_INT_ENABLE_MASK
                         | I2S_CONFIG_RX_OVF_ENABLE_MASK
                         | I2S_CONFIG_RX_FIFO_CLEAR_MASK);
    }
    else
    {
        REG_I2S_CONFIG &= (~(I2S_CONFIG_RX_INT_ENABLE_MASK | I2S_CONFIG_RX_OVF_ENABLE_MASK));
    }
}

void i2s_volume_mute(uint8 enable)
{
//    codec_volume_mute(enable);
}

void i2s_volume_adjust(uint8 volume)
{
//    codec_volume_control(volume);
}

void i2s_rx_volume_adjust(uint8 volume)
{
//    codec_adc_volume_adjust(volume);
}
#endif
