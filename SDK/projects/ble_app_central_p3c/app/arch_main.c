/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ******** ********************************************************************************
 */
 
/*
 * INCLUDES
 ****************************************************************************************
 */
 

#include "rwip_config.h" // RW SW configuration

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "boot.h"      // boot definition
#include "rwip.h"      // RW SW initialization
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#include "timer.h"     // TIMER initialization
#include "icu.h"
#include "flash.h"
#include "uart.h"      	// UART initialization
#if !(BLE_EMB_PRESENT)
//#include "uart2.h"      // UART2 initialization
#endif // !BLE_EMB_PRESENT
#include "flash.h"     // Flash initialization
//#include "led.h"       // Led initialization
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "rf.h"        // RF initialization
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT

#if (BLE_APP_PRESENT)
#include "app.h"       // application functions
#endif // BLE_APP_PRESENT

#if (PLF_NVDS)
#include "nvds.h"         // NVDS definitions
#endif // PLF_NVDS

#include "reg_assert_mgr.h"
#include "BK3435_reg.h"
#include "RomCallFlash.h"
#include "gpio.h"
#include "pwm.h"
#include "audio.h"
#include "app_sdp.h"
#include "app_task.h"
#include "ir.h"
#include "oads.h"
#include "enc_key.h"
#include "wdt.h"
#include "user_config.h"
#include "lld.h"

#if USB_DRIVER
extern void usb_init(int);
extern volatile uint16_t audio_full_flag;
extern unsigned char ucAudioRecordOnOff;
extern volatile uint8_t b_isTRxing;
#endif
/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Creation of uart external interface api
struct rwip_eif_api uart_api;

extern void code_sanity_check(void);

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

static void Stack_Integrity_Check(void);


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void assert_err(const char *condition, const char * file, int line)
{
	uart_printf("%s,condition %s,file %s,line = %d\r\n",__func__,condition,file,line);

}

void assert_param(int param0, int param1, const char * file, int line)
{
	uart_printf("%s,param0 = 0x%x,param1 = 0x%x,file = %s,line = %d\r\n",__func__,param0,param1,file,line);

}

void assert_warn(int param0, int param1, const char * file, int line)
{
	uart_printf("%s,param0 = 0x%x,param1 = 0x%x,file = %s,line = %d\r\n",__func__,param0,param1,file,line);

}

void dump_data(uint8_t* data, uint16_t length)
{
	uart_printf("%s,data = 0x%x,length = 0x%x,file = %s,line = %d\r\n",__func__,data,length);

}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void platform_reset(uint32_t error)
{
    extern void usb_deinit(void);
	UART_PRINTF("error = %x\r\n", error);
	// Disable interrupts
	GLOBAL_INT_STOP();
    
	// Wait UART transfer finished
	uart_finish_transfers();

    if(error==0x1515)
    {        
        usb_deinit();
    }

	if(error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
	{
		// Not yet supported
	}
	else
	{
	    mHWreg_Assign_XVR_Regsiter(09, 0x73203C00);//0x74203C0a
		wdt_enable(10);
		while(1);
	}
}


void bdaddr_env_init(void)
{
	struct bd_addr co_bdaddr;
	flash_read_data(&co_bdaddr.addr[0],0x400e3,6);
	if(co_bdaddr.addr[0]!=0xff ||co_bdaddr.addr[1]!=0xff||
	        co_bdaddr.addr[2]!=0xff||co_bdaddr.addr[3]!=0xff||
	        co_bdaddr.addr[4]!=0xff||co_bdaddr.addr[5]!=0xff )
	{
		memcpy(&co_default_bdaddr,&co_bdaddr,6);
	}
}



//用来加密的原始数据，需要在烧录代码的时候提供
//0102210630355cff0078b69d5538dd22
uint8_t encrypt_key_array[16] =
{
	0x01, 0x02, 0x21, 0x06,
	0x30, 0x35, 0x5c, 0xff,
	0x00, 0x78, 0xb6, 0x9d,
	0x55, 0x38, 0xdd, 0x22
};


/**
 *******************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 *******************************************************************************
 */
extern struct rom_env_tag rom_env;
uint8_t system_mode = RW_NO_MODE;


extern void uart_stack_register(void* cb);
void rwip_eif_api_init(void);
void rw_main(void)
{
    /*
    ***************************************************************************
    * Platform initialization
    ***************************************************************************
    */
#if SYSTEM_SLEEP
    uint8_t sleep_type = 0;
#endif

    //get System sleep flag
    system_sleep_init();

	app_dir_read_notify_init();

    // Initialize the exchange memory interface
    emi_init();

    rwip_eif_api_init();

    // Initialize the Interrupt Controller
    intc_init();
    // Initialize UART component
    gpio_init();

    uart_init(115200);

    uart_stack_register(uart_printf);

    flash_advance_init();
    bdaddr_env_init();

    // Initialize random process
    srand(co_default_bdaddr.addr[0]+co_default_bdaddr.addr[5]);

#if 0
    code_sanity_check();
#endif

#if (NVDS_SUPPORT)
    // Initialize NVDS module
    struct nvds_env_tag env;
    env.flash_read = &flash_read;
    env.flash_write = &flash_write;
    env.flash_erase = &flash_erase;
    nvds_init(env);
#endif

    rom_env_init(&rom_env);


    /*
    ***************************************************************************
    * RW SW stack initialization
    ***************************************************************************
    */
    // Initialize RW SW stack
    rwip_init(0);

    icu_init();


    flash_init();
#if USB_DRIVER 
    usb_init(1);  ///frank 190727
#endif

    REG_AHB0_ICU_INT_ENABLE |= (0x01 << 15); //BLE INT
    REG_AHB0_ICU_IRQ_ENABLE = 0x03;

    switch_clk(MCU_CLK_48M);
    sbc_decoder_init(&sbc_decoder);
    // finally start interrupt handling
    GLOBAL_INT_START();

    // set max scan numbers
    appm_set_max_scan_nums(20);


    UART_PRINTF("start=%x\r\n",EM_BLE_END);

    lld_con_num_set(BLE_CONNECTION_MAX);

    /*
    ***************************************************************************
    * Main loop
    ***************************************************************************
    */
    while(1)
    {
        //schedule all pending events
        rwip_schedule();

        #if USB_DRIVER
        if (ucAudioRecordOnOff == 1)
        {
            b_isTRxing = 0;
            ucAudioRecordOnOff = 0x81;
            UART_PRINTF("ISOC on\r\n");
        }
        else if (ucAudioRecordOnOff == 0)
        {
            ucAudioRecordOnOff = 0x80;
            UART_PRINTF("ISOC off\r\n");
        }
        #endif
        // Checks for sleep have to be done with interrupt disabled
        GLOBAL_INT_DISABLE();

        if(wdt_disable_flag==1)
        {
            wdt_disable();
        }
       
        
        #if SYSTEM_SLEEP
        // Check if the processor clock can be gated
        sleep_type = rwip_sleep();
        if((sleep_type & RW_MCU_DEEP_SLEEP) == RW_MCU_DEEP_SLEEP)
        {
            // 1:idel  0:reduce voltage
            if(icu_get_sleep_mode())
            {
                cpu_idle_sleep();
            }
            else
            {
                cpu_reduce_voltage_sleep();
                cpu_wakeup();
            }
        }
        else if((sleep_type & RW_MCU_IDLE_SLEEP) == RW_MCU_IDLE_SLEEP)
        {
            cpu_idle_sleep();
        }
        #endif
        Stack_Integrity_Check();
        GLOBAL_INT_RESTORE();
    }
}


void rwip_eif_api_init(void)
{
	uart_api.read = &uart_read;
	uart_api.write = &uart_write;
	uart_api.flow_on = &uart_flow_on;
	uart_api.flow_off = &uart_flow_off;
}

const struct rwip_eif_api* rwip_eif_get(uint8_t type)
{
	const struct rwip_eif_api* ret = NULL;
	switch(type)
	{
	case RWIP_EIF_AHI:
	{
		ret = &uart_api;
	}
	break;
#if (BLE_EMB_PRESENT) || (BT_EMB_PRESENT)
	case RWIP_EIF_HCIC:
	{
		ret = &uart_api;
	}
	break;
#elif !(BLE_EMB_PRESENT) || !(BT_EMB_PRESENT)
	case RWIP_EIF_HCIH:
	{
		ret = &uart_api;
	}
	break;
#endif
	default:
	{
		ASSERT_INFO(0, type, 0);
	}
	break;
	}
	return ret;
}

static void Stack_Integrity_Check(void)
{
	if ((REG_PL_RD(STACK_BASE_UNUSED)!= BOOT_PATTERN_UNUSED))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_UNUSED fail!\r\n");
		}
	}

	if ((REG_PL_RD(STACK_BASE_SVC)!= BOOT_PATTERN_SVC))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_SVC fail!\r\n");
		}
	}

	if ((REG_PL_RD(STACK_BASE_FIQ)!= BOOT_PATTERN_FIQ))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_FIQ fail!\r\n");
		}
	}

	if ((REG_PL_RD(STACK_BASE_IRQ)!= BOOT_PATTERN_IRQ))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_IRQ fail!\r\n");
		}
	}

}


void rom_env_init(struct rom_env_tag *api)
{
	memset(&rom_env,0,sizeof(struct rom_env_tag));
	rom_env.prf_get_id_from_task = prf_get_id_from_task;
	rom_env.prf_get_task_from_id = prf_get_task_from_id;
	rom_env.prf_init = prf_init;
	rom_env.prf_create = prf_create;
	rom_env.prf_cleanup = prf_cleanup;
	rom_env.prf_add_profile = prf_add_profile;
	rom_env.rwble_hl_reset = rwble_hl_reset;
	rom_env.rwip_reset = rwip_reset;
#if SYSTEM_SLEEP
	rom_env.rwip_prevent_sleep_set = rwip_prevent_sleep_set;
	rom_env.rwip_prevent_sleep_clear = rwip_prevent_sleep_clear;
	rom_env.rwip_sleep_lpcycles_2_us = rwip_sleep_lpcycles_2_us;
	rom_env.rwip_us_2_lpcycles = rwip_us_2_lpcycles;
	rom_env.rwip_wakeup_delay_set = rwip_wakeup_delay_set;
#endif
	rom_env.platform_reset = platform_reset;
	rom_env.assert_err = assert_err;
	rom_env.assert_param = assert_param;
	rom_env.Read_Uart_Buf = Read_Uart_Buf;
	rom_env.uart_clear_rxfifo = uart_clear_rxfifo;

}

/// @} DRIVERS
