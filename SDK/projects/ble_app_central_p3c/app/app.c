/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "rwapp_config.h"
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition
#include "ke_timer.h"

#if (BLE_APP_FFF0)
#include "app_fff0.h"                 // Application security Definition
#endif // (BLE_APP_FFF0)

#if (BLE_APP_FFE0)
#include "app_ffe0.h"                 // Application security Definition
#endif // (BLE_APP_FFE0)

#if (BLE_APP_SEC)
#include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
#include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)

#if (DISPLAY_SUPPORT)
#include "app_display.h"             // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_OADS)
#include "app_oads.h"                 // Application oads Definition
#endif // (BLE_APP_OADS)

#ifdef BLE_APP_AM0
#include "am0_app.h"                 // Audio Mode 0 Application
#endif //defined(BLE_APP_AM0)

#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)
#include "rf.h"
#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "user_config.h"
#include "sdp_service.h"

/*
 * DEFINES
 ****************************************************************************************
 */
 

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * ENUMERATIONS
 ****************************************************************************************
 */
 
/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
static const struct ke_task_desc TASK_DESC_APP = {NULL, &appm_default_handler,
                                                  appm_state, APPM_STATE_MAX, APP_IDX_MAX};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_init()
{

    uint8_t counter;

    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APPM_INIT);

    // generate a new IRK
    for (counter = 0; counter < KEY_LEN; counter++)
    {
        app_env.loc_irk[counter]    = (uint8_t)co_rand_word();
    }

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/
    #if (BLE_APP_SEC)
    // Security Module
    app_sec_init();
    #endif // (BLE_APP_SEC)

    sdp_service_init();
		
}

extern uint8_t dir_read_notify_flag;
void app_dir_read_notify_init(void)
{
	#if APP_DIR_READ_NOTIFY
	dir_read_notify_flag = 1;
	#else
	dir_read_notify_flag = 0; 
	#endif
}

void appm_get_conn_rssi(uint8_t conidx)
{
	// connection index has been put in addr_src
	struct gapc_get_info_cmd* info_cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
                                            	        KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
                                            	        gapc_get_info_cmd);
	
	// request peer device name.
	info_cmd->operation = GAPC_GET_CON_RSSI;

	// send command
	ke_msg_send(info_cmd);
}


uint8_t appm_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}
uint8_t appm_get_con_dev_num(void)
{
    uint8_t num = 0;
    for(uint8_t i = 0; i < APP_IDX_MAX;i++)    
    {
        if(GAP_INVALID_CONHDL != gapc_get_conhdl(i))
        {
            num++;
        }       
    }
    
    return num;
}

#if USB_DRIVER
#define TOTAL_BLOCK_NUM     100
#define DECODE_VOICE_DATA_LEN  32

//Defines an array to store audio data that has been encoded
uint8_t decode_voice_data[TOTAL_BLOCK_NUM][DECODE_VOICE_DATA_LEN];
//reaord index
uint16_t pbuff_write = 0;
//read index
uint16_t pbuff_read = 0;


/*******************************************************************************
 * Function: read_decode_data
 * Description: read encoded data from loop buffer
 * Input: uint8_t*
 * Output: void
 * Return: uint8_t
 * Others: void
*******************************************************************************/
uint8_t  read_decode_data(uint8_t *buf)
{
	//Read 20 encode data from loop buffer to designated buffer 
	//if(pbuff_write != pbuff_read && ( ((pbuff_write - pbuff_read) >= 2) || ((pbuff_read - pbuff_write )>= 2)))
	
	if(pbuff_write != pbuff_read)
	{
		memcpy(buf,decode_voice_data[pbuff_read],DECODE_VOICE_DATA_LEN );
	//	pbuff_read = ((pbuff_read + 1 )% TOTAL_BLOCK_NUM);
	//	buf+=DECODE_VOICE_DATA_LEN;
	//	memcpy(buf,decode_voice_data[pbuff_read],DECODE_VOICE_DATA_LEN );
		//Update the buffer index of the data 
		//(in fact, the index is between 0-79)
		pbuff_read = ((pbuff_read + 1 )% TOTAL_BLOCK_NUM);
		
		//uart_printf("read 0x%x\r\n",pbuff_read);
		return 1;
	}else
	{
		//uart_printf("buff empty!!0x%x\r\n",pbuff_read);
		return 0;
	} 
}


/*******************************************************************************
 * Function: store_decode_data
 * Description: store encoded data into loop buffer
 * Input: uint8_t*
 * Output: void
 * Return: uint8_t
 * Others: void
*******************************************************************************/
uint8_t store_decode_data(uint8_t *buf,uint8_t store_num)
{
	uint16_t free_cnt;
	uint8_t status ;
	
	//Calculates the number of empty buffer in the circular queue (the 
	//data stored in this queue is encoded)
	if(pbuff_write >= pbuff_read)
	{
		free_cnt = (TOTAL_BLOCK_NUM - pbuff_write + pbuff_read);
	}else
	{
		free_cnt = pbuff_read - pbuff_write;
	}
//	UART_PRINTF("free cnt: %d\r\n", free_cnt);
	
	//If there are at least two empty buffer in the loop queue, the current 
	//encoded data will be stored in buffer. 
	if(free_cnt >= store_num) 
	{
		
		for(uint8_t i = 0;i < store_num;i++)
		{
			memcpy(decode_voice_data[pbuff_write],buf,DECODE_VOICE_DATA_LEN);		
		//Update the buffer index of the data 
		//(in fact, the index is between 0-99)
			pbuff_write = ((pbuff_write + 1 )% TOTAL_BLOCK_NUM);
			buf+= DECODE_VOICE_DATA_LEN;
			
		}
		
		status = 1;
		//UART_PRINTF("buff write 0x%x,0x%x!!!\r\n",buf[0],pbuff_write);
	}else
	{
		
		//UART_PRINTF("buff full 0x%x!!!\r\n",pbuff_write); // for test show
		status = 0;
	}
		
	return status;
}


SbcDecoderContext sbc_decoder;
uint8_t decode_vocie_data(const uint8_t * in_data,uint8_t in_len)
{
	
	uint8_t  sbc_stream[24];
	//uint8_t *p = (uint8_t *)sbc_decoder.pcm_sample;
	sbc_stream[0] = 0xAD;
	sbc_stream[1] = 0x31;
	sbc_stream[2] = 0x0C;
	sbc_stream[3] = 0x7D;
	memcpy(sbc_stream + 4, in_data, 19);

	
	if(sbc_decoder_frame_decode(&sbc_decoder, sbc_stream, sizeof(sbc_stream)) > 0)
	{
		//	Beken_Gpio_Targ(GPIOD_0);
		store_decode_data((uint8_t *)sbc_decoder.pcm_sample,5);
		 
	}else
	{
		UART_PRINTF("decoder fail !!\r\n"); // for test show
	}

	return 0;
}

#endif




#endif //(BLE_APP_PRESENT)

/// @} APP


