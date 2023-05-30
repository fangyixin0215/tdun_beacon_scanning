/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief UART driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup UART
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include "timer.h"      // timer definition
#include "uart.h"       // uart definition
#include "BK3435_reg.h"
#include "reg_uart.h"   // uart register
#include "rwip.h"       // SW interface
#include "h4tl.h"
#include "nvds.h"       // NVDS
#include "rf.h"
#include "dbg.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Max baudrate supported by this UART (in bps)
#define UART_BAUD_MAX      		  3500000
/// Min baudrate supported by this UART (in bps)
#define UART_BAUD_MIN      		  9600

/// Duration of 1 byte transfer over UART (10 bits) in us (for 921600 default baudrate)
#define UART_CHAR_DURATION        11

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */



/*
* common.c
*
*  Created on: 2013-5-7
*  Author: pujie
*/

#include  <stdarg.h>
#include  <stdio.h>
#include "BK3435_reg.h"
#include "ea.h"


#define Uart_Write_Byte(v)               (REG_APB3_UART_PORT=v)

#define UART_TX_FIFO_COUNT               (REG_APB3_UART_FIFO_STAT&0xff)
#define UART_RX_FIFO_COUNT               ((REG_APB3_UART_FIFO_STAT>>8)&0xff)
#define UART_TX_FIFO_FULL                (REG_APB3_UART_FIFO_STAT&0x00010000)
#define UART_TX_FIFO_EMPTY               (REG_APB3_UART_FIFO_STAT&0x00020000)
#define UART_RX_FIFO_FULL                (REG_APB3_UART_FIFO_STAT&0x00040000)
#define UART_RX_FIFO_EMPTY               (REG_APB3_UART_FIFO_STAT&0x00080000)
#define UART_TX_WRITE_READY              (REG_APB3_UART_FIFO_STAT&0x00100000)
#define UART_RX_READ_READY               (REG_APB3_UART_FIFO_STAT&0x00200000)
#define bit_UART_TXFIFO_NEED_WRITE        0x01
#define bit_UART_RXFIFO_NEED_READ         0x02
#define bit_UART_RXFIFO_OVER_FLOW         0x04
#define bit_UART_RX_PARITY_ERROR          0x08
#define bit_UART_RX_STOP_ERROR            0x10
#define bit_UART_TX_PACKET_END            0x20
#define bit_UART_RX_PACKET_END            0x40
#define bit_UART_RXD_WAKEUP_DETECT        0x80


#define uart_tx_en    0x1      // 0: Disable TX, 1: Enable TX 
#define uart_rx_en    0x1      // 0: Disable RX, 1: Enable RX
#define irda_mode     0x0      // 0: UART  MODE, 1: IRDA MODE
#define data_len      0x3      // 0: 5 bits, 1: 6 bits, 2: 7 bits, 3: 8 bits
#define parity_en     0x0      // 0: NO Parity, 1: Enable Parity
#define parity_mode   0x1      // 0: Odd Check, 1: Even Check 
#define stop_bits     0x0      // 0: 1 stop-bit, 1: 2 stop-bit 
#define uart_clks     16000000 // UART's Main-Freq, 26M 
#define baud_rate     115200   // UART's Baud-Rate,  1M 


unsigned char uart_rx_buf[UART0_RX_FIFO_MAX_COUNT];
unsigned char uart_tx_buf[UART0_TX_FIFO_MAX_COUNT];
volatile bool uart_rx_done = 0;
volatile unsigned long uart_rx_index = 0;
uint8_t cur_read_buf_idx = 0;

extern uint8_t system_mode;
#define UART_READ_BYTE()                 ((REG_APB3_UART_PORT>>8)&0xff)


///UART Character format
enum UART_CHARFORMAT
{
    UART_CHARFORMAT_8 = 0,
    UART_CHARFORMAT_7 = 1
};

///UART Stop bit
enum UART_STOPBITS
{
    UART_STOPBITS_1 = 0,
    UART_STOPBITS_2 = 1  /* Note: The number of stop bits is 1.5 if a character format
                            with 5 bit is chosen*/
};

///UART Parity enable
enum UART_PARITY
{
    UART_PARITY_DISABLED = 0,
    UART_PARITY_ENABLED  = 1
};

///UART Parity type
enum UART_PARITYBIT
{
    UART_PARITYBIT_EVEN  = 0,
    UART_PARITYBIT_ODD   = 1,
    UART_PARITYBIT_SPACE = 2, // The parity bit is always 0.
    UART_PARITYBIT_MARK  = 3  // The parity bit is always 1.
};

///UART HW flow control
enum UART_HW_FLOW_CNTL
{
    UART_HW_FLOW_CNTL_DISABLED = 0,
    UART_HW_FLOW_CNTL_ENABLED = 1
};

///UART Input clock select
enum UART_INPUT_CLK_SEL
{
    UART_INPUT_CLK_SEL_0 = 0,
    UART_INPUT_CLK_SEL_1 = 1,
    UART_INPUT_CLK_SEL_2 = 2,
    UART_INPUT_CLK_SEL_3 = 3
};

///UART Interrupt enable/disable
enum UART_INT
{
    UART_INT_DISABLE = 0,
    UART_INT_ENABLE = 1
};

///UART Error detection
enum UART_ERROR_DETECT
{
    UART_ERROR_DETECT_DISABLED = 0,
    UART_ERROR_DETECT_ENABLED  = 1
};

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
struct uart_txrxchannel
{
	/// call back function pointer
	void (*callback) (void*, uint8_t);
	/// Dummy data pointer returned to callback when operation is over.
	void* dummy;
};

/// UART environment structure
struct uart_env_tag
{
	/// tx channel
	struct uart_txrxchannel tx;
	/// rx channel
	struct uart_txrxchannel rx;
	/// error detect
	uint8_t errordetect;
	/// external wakeup
	bool ext_wakeup;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// uart environment structure
struct uart_env_tag uart_env;
char uart_buff[255];



static UART_RX_CALLBACK_T usrt_rx_cb = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint8_t Read_Uart_Buf(void)
{
	return uart_rx_buf[cur_read_buf_idx++];
}

uint8_t Uart_Read_Byte(void)
{
	return (REG_APB3_UART_PORT&0xff);
}


int uart_putchar(char * st)
{
	uint8_t num = 0;
	while (*st)
	{
		if(UART_TX_WRITE_READY)
		{
			REG_APB3_UART_PORT = *st;
			st++;
			num++;
		}
	}
	return num;
}

int uart_printf(const char *fmt,...)
{
#if (UART_PRINTF_EN && UART_DRIVER)
	int n;

	va_list ap;
	va_start(ap, fmt);
	n=vsprintf(uart_buff, fmt, ap);
	va_end(ap);
	uart_putchar(uart_buff);
	if(n > sizeof(uart_buff))
	{
		uart_putchar("buff full \r\n");
	}

	return n;
#else
    return 0;
#endif
}

int uart_printf_null(const char *fmt,...)
{
	return 0;
}


char *hex2Str( uint8_t data)
{

	char hex[] = "0123456789ABCDEF";
	static char str[3];
	char *pStr = str;

	*pStr++ = hex[data >> 4];
	*pStr++ = hex[data & 0x0F];
	*pStr = 0;

	return str;
}

void uart_print_int(unsigned int num)
{
#if UART_PRINTF_EN
	uint8_t i;
	uint8_t m;

	uart_putchar((char *)"0x");
	for (i = 4; i > 0; i--)
	{
		m = ((num >> (8 * (i - 1)))& 0xff);
		uart_putchar(hex2Str(m));
	}
	uart_putchar("\r\n");
#endif    
}



void cpu_delay( volatile unsigned int times)
{
	while(times--)
	{
		for(uint32_t i = 0; i < 1000; i++)
			;
	}
}


void uart_init(uint32_t baudrate)
{
#if (UART_DRIVER)
	unsigned int baud_divisor ;
	REG_AHB0_ICU_UARTCLKCON   &= ~(0x1 << 0) ;  // Enable Uart's Clocks
	
	baud_divisor=16000000/baudrate-1;
	REG_APB3_UART_CFG  = (baud_divisor<<8) +
	                     (stop_bits   <<7) +
	                     (data_len    <<3) +
	                     (irda_mode   <<2) +
	                     (uart_rx_en  <<1) +
	                     uart_tx_en ;

	REG_APB3_UART_FIFO_CFG = (1<<BIT_TX_FIFO_THRESHOLD)|(16<<BIT_RX_FIFO_THRESHOLD)|(0x2 << BIT_STOP_DETECT_TIME);//(0x3 << BIT_STOP_DETECT_TIME);
	REG_APB3_UART_INT_ENABLE = ((0x01 << 1) | (0x01 << 6) | (0x01 << 7));
	REG_APB3_UART_FLOW_CFG  = 0x00000000 ;  // No Flow Control
	REG_APB3_UART_WAKE_CFG  = ((0x01 << 0 )| (0x01 << 20) |  (0x01 << 21)| (0x01 << 22));  // No Wake Control

	REG_APB5_GPIOA_CFG  &= ~((0x3<<BIT_GPIO_PULL_UP)  + (0x3<<BIT_GPIO_PERI_EN));
	REG_APB5_GPIOA_CFG  |= ((0x3<<BIT_GPIO_PULL_UP));
	REG_APB5_GPIOA_CFG  |=   (0x3<<BIT_GPIO_OUT_EN_N);

	REG_APB5_GPIOA_DATA &= ~ (0x3<<BIT_GPIO_INPUT_EN);


	uart_env.rx.callback = NULL;
	uart_env.rx.dummy    = NULL;

	//REG_AHB0_ICU_INT_ENABLE |= (0x01 << 5);
#endif
}



void uart_flow_on(void)
{
	// Configure modem (HW flow control enable)
	// uart_flow_en_setf(0);
}

void uart_clear_rxfifo(void)
{

	while(uart_rx_fifo_rd_ready_getf())
	{
		Uart_Read_Byte();
	}
	memset(uart_rx_buf,0,UART0_RX_FIFO_MAX_COUNT);

}
bool uart_flow_off(void)
{

	return true;

}

void uart_finish_transfers(void)
{
	uart_flow_en_setf(1);

	// Wait TX FIFO empty
	while(!uart_tx_fifo_empty_getf());
}


void uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	ASSERT_ERR(bufptr != NULL);
	ASSERT_ERR(size != 0);
	ASSERT_ERR(callback != NULL);
	uart_env.rx.callback = callback;

	uart_env.rx.dummy    = dummy;

}



void uart_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	ASSERT_ERR(bufptr != NULL);
	ASSERT_ERR(size != 0);
	ASSERT_ERR(callback != NULL);

	uint8_t len;
	len = size;

	uart_env.tx.callback = callback;
	uart_env.tx.dummy    = dummy;

	//Delay_ms(100);
	while(len--)
	{
		//cpu_delay(10);
		if(UART_TX_WRITE_READY)
		{
			REG_APB3_UART_PORT = *bufptr;
			bufptr++;
		}
	}

	if(callback != NULL)
	{
		// Clear callback pointer
		uart_env.tx.callback = NULL;
		uart_env.tx.dummy    = NULL;
		// Call handler
		callback(dummy, RWIP_EIF_STATUS_OK);
	}
}

static void uart_send_byte(unsigned char data)
{
	while (!uart_tx_fifo_empty_getf());

	REG_APB3_UART_PORT = data ;
}

void uart_send(unsigned char *buff, int len)
{
	while (len--)
		uart_send_byte(*buff++);
}

void uart_cb_register(UART_RX_CALLBACK_T cb)
{
	if(cb)
	{
		usrt_rx_cb = cb;
	}
}

void uart_cb_clear(void)
{
	if(usrt_rx_cb)
	{
		usrt_rx_cb = NULL;
	}
}

volatile uint8_t uart_dut_reg_flag = 0;
extern void pn9_test_process(uint8_t* buffer, uint8_t len);

enum
{
    NORMAL_MODE = 0,
    DUT_MODE = 1,
    REG_MODE = 2,
};

#define DEBUG_RF_REG				1
extern volatile uint32_t XVR_ANALOG_REG_BAK[16];

void uart_rx_cmd_respone(uint8_t *buff,uint8_t len )
{
    uint8_t respone_buff[32];

    respone_buff[0]=0x04;
    respone_buff[1]=0x0e;
    respone_buff[2]=4+len;
    respone_buff[3]=0x01;
    respone_buff[4]=0xe0;
    respone_buff[5]=0xfc;
    respone_buff[6]=len;

    memcpy(&respone_buff[7],buff,len);

    uart_send(respone_buff,7+len);


}

void uart_rx_cmd_handler(uint8_t *buff,uint8_t len )
{
    //uint32_t rx_command;
    uint8_t length;
    //static uint8_t rx_cmd_count=0;
    length=len;
    switch(buff[0])
    {
        case UART_LINK_CHECK:
        {
            if(length==1)
            {
              //  if(rx_cmd_count++>2)
              //      uart_env.uart_download_state = 1;
                buff[1]=0;
            }
        }
        break;
        case CMD_SYS_RESET:
        {
        /*    if(length==5)
            {
                rx_command = (buff[1]<<24|buff[2]<<16|buff[3]<<8|buff[4]);
                if(rx_command==0x95279527)
                {
                    cpu_reset();

                }
            }*/
            length -=1;
        }
        break;
        #if(DEBUG_RF_REG)
        case REG_WRITE_CMD:
        {
            uint32_t reg_index;
            uint32_t reg_addr;
            uint32_t reg_value;

            reg_addr=buff[1]+(buff[2]<<8)+(buff[3]<<16)+(buff[4]<<24);
            reg_value=buff[5]+(buff[6]<<8)+(buff[7]<<16)+(buff[8]<<24);
            reg_index = (reg_addr-APB_XVER_BASE)/4;
         //   UART_PRINTF("w=%x,%x\r\n",reg_addr,reg_value);

            *(volatile uint32_t *)reg_addr = reg_value;

            if(reg_index<=0x0f)
            {
                XVR_ANALOG_REG_BAK[reg_index] = reg_value;
            }

        }
        break;
        case REG_READ_CMD:
        {
            uint32_t reg_index;
            uint32_t reg_addr;
            uint32_t reg_value;

            reg_addr=buff[1]+(buff[2]<<8)+(buff[3]<<16)+(buff[4]<<24);

            reg_index = (reg_addr-APB_XVER_BASE)/4;

            if (reg_index<=0x0f)
            {
                reg_value  = XVR_ANALOG_REG_BAK[reg_index];
            }
            else
            {
                reg_value  = *(volatile uint32_t *)reg_addr;
            }
         //   UART_PRINTF("r=%x,%x\r\n",reg_addr,reg_value,reg_index);

            memcpy(&buff[5],&reg_value,4);
            length += 3;

        }
        break;
        #endif
    }
  //  if(buff[0]!=0)
  //      uart_env.uart_download_state=0;

    #if(DEBUG_RF_REG)
    uart_rx_cmd_respone(&buff[1],length);
    #endif

}
void uart_isr(void)
{
    static uint8_t dut_test_cmd;
    uint32_t IntStat;

    IntStat = uart_isr_stat_get();
    if(uart_rx_fifo_need_rd_isr_getf() || uart_rx_end_isr_getf()|| uart_rxd_wakeup_isr_getf())
    {
		    while((REG_APB3_UART_FIFO_STAT & (0x01 << 21)))
		    {
            uart_rx_buf[uart_rx_index++] = UART_READ_BYTE();
            if( UART0_RX_FIFO_MAX_COUNT == uart_rx_index )
            {
                uart_rx_index = 0;
            }
        }
    }
    if((system_mode & RW_DUT_MODE) == RW_DUT_MODE)
    {
        void (*callback) (void*, uint8_t) = NULL;
        void* data =NULL;
        if( uart_rx_end_isr_getf())
        {
            if((uart_rx_buf[0] == 0x01) &&(uart_rx_buf[1] == 0xe0)&& (uart_rx_buf[2] == 0xfc)){
                if((uart_rx_buf[3] == 0x02)&& (uart_rx_buf[4] == 0xaf) &&(uart_rx_buf[5] == 0x01)){
                    ldo_mode_set();
                }else if((uart_rx_buf[3] == 0x02)&& (uart_rx_buf[4] == 0xaf) &&(uart_rx_buf[5] == 0x02)){
                    buck_mode_set();
                }else{

                }
            uart_dut_reg_flag = REG_MODE;
            }
            else if((uart_rx_buf[0] == 0x01) && (uart_rx_index > 3)){
                uart_dut_reg_flag = DUT_MODE;
            }
			
            if(uart_dut_reg_flag != NORMAL_MODE)
            {
                uart_rx_done = 1 ;
            }
            else
            {
                uart_rx_index = 0;
            }
        }
        if(uart_dut_reg_flag == REG_MODE)
        {
            //   UART_PRINTF("%x,%x,%x,%x,%x,%x,%x\r\n",uart_rx_buf[0],uart_rx_buf[1],uart_rx_buf[2],uart_rx_buf[3],uart_rx_buf[4],uart_rx_buf[5],uart_rx_buf[6]);
            uart_rx_cmd_handler(&uart_rx_buf[4] ,uart_rx_buf[3]);
            uart_rx_index = 0;
        }
        if(uart_dut_reg_flag == DUT_MODE)
        {
            uart_rx_index = 0;
            //Abel disable 20220407, dut test process may appear two identical commands consecutively
            //if(dut_test_cmd!=uart_rx_buf[1])  
            {
                if(((dut_test_cmd==0x1d)&&(uart_rx_buf[1]==0x1e))||((dut_test_cmd==0x1e)&&(uart_rx_buf[1]==0x1d)))
                {

                }
                else
                {
                    dut_test_cmd=uart_rx_buf[1]; 
                    callback = uart_env.rx.callback;
                    data = uart_env.rx.dummy;
                    if(callback != NULL)
                    {
                        // Clear callback pointer
                        uart_env.rx.callback = NULL;
                        uart_env.rx.dummy    = NULL;
                        callback(data, RWIP_EIF_STATUS_OK);
                    }
                }
            }
        }
        uart_dut_reg_flag = NORMAL_MODE;
    }
    else if((system_mode & RW_FCC_MODE) == RW_FCC_MODE)
    {
        uart_rx_done = 1;
    }
    else if((system_mode & RW_PN9_MODE) == RW_PN9_MODE)
    {
        uart_rx_done = 1;
        pn9_test_process(uart_rx_buf,uart_rx_index);
        uart_rx_index = 0;
    }
    else
    {
        uart_rx_done = 1;
        if(usrt_rx_cb)
        {
            (*usrt_rx_cb)(uart_rx_buf, uart_rx_index);
        }
        uart_rx_index = 0;
    }
    uart_isr_stat_set(IntStat);
}

uint8_t check_uart_stop(void)
{
	return uart_tx_fifo_empty_getf();
}

/// @} UART
