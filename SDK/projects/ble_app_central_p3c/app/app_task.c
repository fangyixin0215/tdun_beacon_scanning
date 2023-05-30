/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions

#include "ke_timer.h"             // Kernel timer
//#include "ke_task.h"

#if (BLE_APP_FFF0)
#include "app_fff0.h"              // fff0 Module Definition
#include  "fff0s_task.h"
#endif //(BLE_APP_FFF0)


#if (BLE_APP_FFE0)
#include "app_ffe0.h"              // ffe0 Module Definition
#endif //(BLE_APP_FFE0)
#include "flash.h"

#if (BLE_APP_SEC)
#include "app_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"               // Health Thermometer Module Definition
#include "htpt_task.h"
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
#include "app_hid.h"              // HID Module Definition
#include "hogpd_task.h"
#endif //(BLE_APP_HID)

#ifdef BLE_APP_AM0
#include "am0_app.h"             // Audio Mode 0 Application
#endif //defined(BLE_APP_AM0)

#if (BLE_APP_OADS)
#include "app_oads.h"
#include "oads_task.h"
#endif //(BLE_APP_OADS)

#if (DISPLAY_SUPPORT)
#include "app_display.h"         // Application Display Definition
#endif //(DISPLAY_SUPPORT)
#include "gpio.h"
#include "audio.h"
#include "uart.h"
#include "BK3435_reg.h"
#include "icu.h"
#include "reg_ble_em_cs.h"
#include "lld.h"
#include "flash.h"
#include "wdt.h"
#include "app_sdp.h"
#include "reg_ble_em_rx_desc.h"
#include "gapc_sig.h"
#include "llc_llcp.h"
#include "nvds.h"
#include "app_wlist.h"
#include "user_config.h"

#include "uart_data.h"
#include "ble_to_MTK_protocol.h"
struct gap_bdaddr  con_bdaddr;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
	// Counter
	uint8_t counter;

	// Get the message handler function by parsing the message table
	for (counter = handler_list->msg_cnt; 0 < counter; counter--)
	{

		struct ke_msg_handler handler = (*(handler_list->msg_table + counter - 1));

		if ((handler.id == msgid) ||
		        (handler.id == KE_MSG_DEFAULT_HANDLER))
		{
			// If handler is NULL, message should not have been received in this state
			ASSERT_ERR(handler.func);

			return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
		}
	}

	// If we are here no handler has been found, drop the message
	return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Application has not been initialized
	ASSERT_ERR(ke_state_get(dest_id) == APPM_INIT);

	// Reset the stack
	struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
	                             TASK_GAPM, TASK_APP,
	                             gapm_reset_cmd);

	cmd->operation = GAPM_RESET;

	ke_msg_send(cmd);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("param->operation = 0x%x, param->status = 0x%x \r\n", param->operation, param->status);
	switch(param->operation)
	{
		// Reset completed
	case (GAPM_RESET):
	{
		if(param->status == GAP_ERR_NO_ERROR)
		{
			struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
			                                      TASK_GAPM, TASK_APP,
			                                      gapm_set_dev_config_cmd);
			// Set the operation
			cmd->operation = GAPM_SET_DEV_CONFIG;
			// Set the device role - Central
			cmd->role      = GAP_ROLE_CENTRAL;
			// Set Data length parameters
			cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
			cmd->sugg_max_tx_time = BLE_MIN_TIME;
			cmd->max_mtu = 185;
			
			// Do not support secure connections
			cmd->pairing_mode = GAPM_PAIRING_LEGACY;
			//
			cmd->att_cfg = 0x80;

			// Send message
			ke_msg_send(cmd);

		}
		else
		{
			ASSERT_ERR(0);
		}
	}
	break;
	case (GAPM_PROFILE_TASK_ADD):
	{
		
	}
	break;
	// Device Configuration updated
	case (GAPM_SET_DEV_CONFIG):
	{
		ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

		UART_PRINTF("gapm set dev config\r\n");

		// Go to the ready state
		ke_state_set(TASK_APP, APPM_READY);

		#if (ADDR_CHANGE_EN)
		co_default_bdaddr.addr[0] += 1;
		ble_bdaddrl_set(co_read32p(&co_default_bdaddr.addr[0]));
		ble_bdaddru_set(co_read16p(&co_default_bdaddr.addr[4]));
		for(int i = 0; i < 6; i++)
		{
			llm_le_env.public_add.addr[i] = co_default_bdaddr.addr[i];
		}
		#endif

#if WHITE_LIST_EN
		//if device bonded, load bond addr into white list
		if(app_sec_get_bond_status())
		{
			uint8_t length = NVDS_LEN_PEER_BD_ADDRESS;           
            struct gap_bdaddr  bond_addr;
			if(nvds_get(NVDS_TAG_PEER_BD_ADDRESS, &length, (uint8_t *)&bond_addr) == NVDS_OK)
			{
				appm_add_dev_to_wlist(bond_addr);
			}
		}
#endif		
		//start scanning
		
        ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,20);
	}
	break;

	case (GAPM_SCAN_ACTIVE):
	case (GAPM_SCAN_PASSIVE):
	{
		if (param->status == GAP_ERR_CANCELED)
		{
			if (ke_state_get(TASK_APP) == APPM_READY)
			{
				UART_PRINTF("scan stop canceled, gapm scan passive ok\r\n");
                ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,50);
			}
			if (ke_state_get(TASK_APP) == APPM_WAIT_SCANNEND)
			{
				//gpio_triger(0x32);
				UART_PRINTF("con canceled, gapm scan passive ok\r\n");
				ke_state_set(TASK_APP, APPM_READY);
				appm_start_connencting(con_bdaddr);
			}
			
		}
	}
	break;

	case (GAPM_CONNECTION_DIRECT):
	{
		if (param->status == GAP_ERR_CANCELED)
		{
			if (ke_state_get(TASK_APP) == APPM_CONNECTING)
			{
				ke_state_set(TASK_APP, APPM_READY);
				UART_PRINTF("canceled gapm start connection cmd\r\n");
			}
		}
	}
	break;

	default:
	{
		// Drop the message
	}
	break;
	}

	return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	switch(param->req)
	{
	case GAPC_DEV_NAME:
	{
		struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
		                                     src_id, dest_id,
		                                     gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
		cfm->req = param->req;
		cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

		// Send message
		ke_msg_send(cfm);
	}
	break;

	case GAPC_DEV_APPEARANCE:
	{
		// Allocate message
		struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
		                                    src_id, dest_id,
		                                    gapc_get_dev_info_cfm);
		cfm->req = param->req;

		// No appearance
		cfm->info.appearance = 0;

		// Send message
		ke_msg_send(cfm);
	}
	break;

	case GAPC_DEV_SLV_PREF_PARAMS:
	{
		// Allocate message
		struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
		                                    src_id, dest_id,
		                                    gapc_get_dev_info_cfm);
		cfm->req = param->req;
		// Slave preferred Connection interval Min
		cfm->info.slv_params.con_intv_min = 8;
		// Slave preferred Connection interval Max
		cfm->info.slv_params.con_intv_max = 10;
		// Slave preferred Connection latency
		cfm->info.slv_params.slave_latency = 0;
		// Slave preferred Link supervision timeout
		cfm->info.slv_params.conn_timeout  = 600; 

		// Send message
		ke_msg_send(cfm);
	}
	break;

	default: /* Do Nothing */
		break;
	}


	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Set Device configuration
	struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
	                                    gapc_set_dev_info_cfm);
	// Reject to change parameters
	cfm->status = GAP_ERR_REJECTED;
	cfm->req = param->req;
	// Send message
	ke_msg_send(cfm);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

uint8_t app_start_encrypt_flag = 0;
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_connection_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s\r\n", __func__);

	app_env.conidx = KE_IDX_GET(src_id);

    UART_PRINTF("app_env.conidx=%x\r\n", app_env.conidx); 
    
	if (app_env.conidx != GAP_INVALID_CONIDX)
	{
		// Retrieve the connection info from the parameters
		app_env.conhdl = param->conhdl;

		struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
										KE_BUILD_ID(TASK_GAPC, app_env.conidx), 
										TASK_APP,
										gapc_connection_cfm);		
	    cfm->auth = GAP_AUTH_REQ_NO_MITM_BOND;
		//Send the message		
		ke_msg_send(cfm);

		//Send MTU update request
		ke_timer_set(APP_GATTC_EXC_MTU_CMD, TASK_APP, 5);

		
#if (APP_GET_RSSI_EN)
		ke_timer_set(APP_GET_RSSI_TIMER,TASK_APP,1000);
#endif

	}
	else
	{
		UART_PRINTF("connect error\r\n");
	}

	return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief gapc_conn_rssi_ind_handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_conn_rssi_ind_handler(ke_msg_id_t const msgid,
        struct gapc_con_rssi_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{

	//if(ke_state_get(dest_id) == APPM_CONNECTED)
	{
		UART_PRINTF("get rssi = %d\r\n",param->rssi);
	}

	return (KE_MSG_CONSUMED);
}


static int app_disconnect_timer_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	UART_PRINTF("%s\r\n",__func__);

	if(ke_state_get(dest_id) == APPM_CONNECTED)
	{
		appm_disconnect();
	}
	
	return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("gapc_cmp_evt_handler operation = %x, status = %x\r\n",param->operation, param->status);
	switch(param->operation)
	{
	case (GAPC_UPDATE_PARAMS):
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			//appm_disconnect();
		}
		UART_PRINTF("gapc update paras state: 0x%x\r\n", param->status);
	}
	break;

	case (GAPC_BOND):
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc bond fail\r\n");
		}
		else
		{
			UART_PRINTF("gapc bond ok\r\n");
            
            #if(SMP_ENCRYPT_EN)
	   		ke_state_set(TASK_APP, APPM_SDP_DISCOVERING);
          	sdp_discover_all_service(app_env.conidx); 
          	#endif
		}
	}
	break;

	case (GAPC_ENCRYPT):
	{
		if (param->status != GAP_ERR_NO_ERROR)
		{
			UART_PRINTF("gapc encrypt fail\r\n");
		}
		else
		{
			UART_PRINTF("gapc encrypt ok,app_env.conidx=%x\r\n",app_env.conidx);
            if(sdp_env_init.used_status[app_env.conidx][0] == FREE_STATUS)
            {
                ke_state_set(TASK_APP, APPM_SDP_DISCOVERING);
                sdp_discover_all_service(app_env.conidx);
            }
            else
            {
                sdp_prf_register_all_atthdl2gatt(app_env.conidx);
                if(ke_timer_active(APP_START_SCAN_TIMER, TASK_APP))
                {
                    ke_timer_clear(APP_START_SCAN_TIMER, TASK_APP);
                }
                ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,200);
                ke_msg_send_basic(APP_PARAM_UPDATE_REQ,TASK_APP,TASK_APP);
            }
		}
	}
	break;

	default:
		break;
	}

	return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{

	UART_PRINTF("disconnect link reason = 0x%x\r\n",param->reason);

	ke_timer_clear(APP_START_SMP_REQ_TIMER,TASK_APP);
	ke_timer_clear(APP_START_ENCRYPT_TIMER,TASK_APP);
	ke_timer_clear(APP_GET_RSSI_TIMER,TASK_APP);
	
	// Go to the ready state
	if((ke_state_get(TASK_APP) != APPM_SCANNING) )
    {   
    	ke_state_set(TASK_APP, APPM_READY);
        ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,20);
    }

	return (KE_MSG_CONSUMED);
}


/*******************************************************************************
 * Function: app_get_rssi_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *		  param -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance (TASK_GAP).
 *		  ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_get_rssi_timer_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{	
	UART_PRINTF("%s\r\n", __func__);
	
	//if(ke_state_get(dest_id) == APPM_CONNECTED)
	{
		appm_get_conn_rssi(app_env.conidx);
	}
	
	ke_timer_set(APP_GET_RSSI_TIMER,TASK_APP,500);

	return KE_MSG_CONSUMED;

}


static int app_start_smpc_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)

{
	UART_PRINTF("%s\r\n", __func__);
	
	app_sec_bond_cmd_req();
	
	return KE_MSG_CONSUMED;	
}


static int app_start_encrypt_handler(ke_msg_id_t const msgid,
                                    struct gapm_profile_added_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	UART_PRINTF("%s\r\n", __func__);
	uint8_t conidx = KE_IDX_GET(src_id);
	if(1)//!app_start_encrypt_flag)
	{
		UART_PRINTF("app set encrypt res status\r\n");
		//llcp_set_encrypt_status();
		app_sec_encry_cmd_req();
		//ke_timer_set(APP_START_ENCRYPT_TIMER,TASK_APP,11);
	}
	else
	{
		if(llcp_get_encrypt_res_status())
		{
			UART_PRINTF("app clear encrypt res status\r\n");
			llcp_clear_encrypt_status();
			
			app_sec_remove_bond(conidx);
			appm_disconnect();
		}
	}
	
	
	return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles profile add indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
        struct gapm_profile_added_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Current State
	uint8_t state = ke_state_get(dest_id);

	if (state == APPM_CREATE_DB)
	{
	
	}
	else
	{
	
	}

	return KE_MSG_CONSUMED;
}



/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
	// Retrieve identifier of the task from received message
	ke_task_id_t src_task_id = MSG_T(msgid);
	// Message policy
	uint8_t msg_pol          = KE_MSG_CONSUMED;

	UART_PRINTF("appm_msg_handler src_task_id = 0x%x\r\n",src_task_id);


	switch (src_task_id)
	{
	case (TASK_ID_GAPC):
	{
#if (SMP_ENCRYPT_EN)
		if ((msgid >= GAPC_BOND_CMD) && (msgid <= GAPC_SECURITY_IND))
		{
			// Call the Security Module
			msg_pol = appm_get_handler(&app_sec_table_handler, msgid, param, src_id);
		}
#endif //(BLE_APP_SEC)
	}
	break;

#if (BLE_APP_SDP)
	case (TASK_ID_SDP):
	{
		// Call the SDP Module
		msg_pol = appm_get_handler(&sdp_default_handler, msgid, param, src_id);
	}
	break;
#endif //(TASK_ID_SDP)		

	default:
		break;
	}

	return (msg_pol);
}


/*******************************************************************************
 * Function: gapc_update_conn_param_req_handler
 * Description: Update request command processing from slaver connection parameters
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_update_conn_param_req_handler (ke_msg_id_t const msgid,
        const struct gapc_param_update_req_ind  *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	
	UART_PRINTF("gapc_update_conn_param_req_handler\r\n");
	struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = 6;
    cmd->intv_max   = 6;
    cmd->latency    = 0;
    cmd->time_out   = 300;

    // not used by a slave device
    cmd->ce_len_min = 2;
    cmd->ce_len_max = 4;
		
    UART_PRINTF("intv_min = %d,intv_max = %d,latency = %d,time_out = %d\r\n",cmd->intv_min,cmd->intv_max,cmd->latency,cmd->time_out);
	
    // Send the message
    ke_msg_send(cmd);

	return KE_MSG_CONSUMED;
}


/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid,
        const struct gapc_le_pkt_size_ind  *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);

	UART_PRINTF("1max_rx_octets = %d\r\n",param->max_rx_octets);
	UART_PRINTF("1max_rx_time = %d\r\n",param->max_rx_time);
	UART_PRINTF("1max_tx_octets = %d\r\n",param->max_tx_octets);
	UART_PRINTF("1max_tx_time = %d\r\n",param->max_tx_time);

	return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid,
        const struct gapc_param_updated_ind  *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);

	UART_PRINTF("con_interval = %d\r\n",param->con_interval);
	UART_PRINTF("con_latency = %d\r\n",param->con_latency);
	UART_PRINTF("sup_to = %d\r\n",param->sup_to);
    //app_env.conidx = KE_IDX_GET(src_id);
    if((ke_state_get(TASK_APP)==APPM_SDP_DISCOVERING)&& (param->con_latency>0))
    {
        ke_msg_send_basic(APP_PARAM_UPDATE_REQ,TASK_APP,TASK_APP);
    }

	return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GATTC_EXC_MTU_CMD
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_exchange_req_handler(ke_msg_id_t const msgid,
        struct gattc_exc_mtu_cmd const *req,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);
	struct gattc_exc_mtu_cmd *cmd = KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
	                                KE_BUILD_ID(TASK_GATTC, app_env.conidx),
	                                TASK_APP,gattc_exc_mtu_cmd);
	cmd->operation = GATTC_MTU_EXCH;
	cmd->seq_num = 0;
	ke_msg_send(cmd);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
        struct gattc_mtu_changed_ind const *ind,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n",__func__);
	UART_PRINTF("ind->mtu = %d.seq = %d\r\n",ind->mtu,ind->seq_num);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   GAPC_PARAM_UPDATE_REQ_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_param_update_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);
	// Prepare the GAPC_PARAM_UPDATE_CFM message
	struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
	                                    src_id, dest_id,
	                                    gapc_param_update_cfm);

	cfm->ce_len_max = 4;
	cfm->ce_len_min = 2;
	cfm->accept = true;

	// Send message
	ke_msg_send(cfm);

	return (KE_MSG_CONSUMED);
}


struct adv_addr_list_t adv_addr_list;
uint8_t con_dev_str_name1[] ="TGVG01";  //P3C项目蓝牙信标名
uint8_t ltk_buffer[36];


static int gapm_adv_report_ind_handler(ke_msg_id_t const msgid,
                                       struct adv_report const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
	uint8_t find1 = 0 ;
	if (ke_state_get(TASK_APP) == APPM_WAIT_SCANNEND)
	{
		return KE_MSG_CONSUMED;
	}
	//UART_PRINTF("%s......................\r\n",__func__);
	// save peer address	
	memcpy(&(adv_addr_list.adv_addr[adv_addr_list.nums]),param->adv_addr.addr,6);	
	adv_addr_list.addr_type[adv_addr_list.nums] = param->adv_addr_type;
	adv_addr_list.nums++;
    
    if(adv_addr_list.nums>=appm_get_max_scan_nums())
    {
        adv_addr_list.nums=0;
        appm_stop_scanning();
        return KE_MSG_CONSUMED;
    }
    app_sec_env.bonded=0;

	//匹配蓝牙名，暂定默认信标蓝牙名
    find1 = appm_adv_data_decode(param->data_len, param->data,con_dev_str_name1,sizeof(con_dev_str_name1)-1);
    
    if(find1 == 1)
	{
		char temp_addr[20]={0};
		p3c_beacon_not_find_count = 0;//表示找到信标
		//UART_PRINTF("\r\n######################################\r\n");
		//UART_PRINTF("rssi = %d\r\n",param->rssi);
		//UART_PRINTF("device number = %d\r\n", adv_addr_list.nums);
		//UART_PRINTF("adv_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n",param->adv_addr.addr[5],param->adv_addr.addr[4],param->adv_addr.addr[3],param->adv_addr.addr[2],param->adv_addr.addr[1],param->adv_addr.addr[0]);
		//UART_PRINTF("BLE adv name:%s\r\n",con_dev_str_name1);
		//UART_PRINTF("######################################\r\n");
		sprintf(temp_addr,"%02x:%02x:%02x:%02x:%02x:%02x",param->adv_addr.addr[5],param->adv_addr.addr[4],param->adv_addr.addr[3],param->adv_addr.addr[2],param->adv_addr.addr[1],param->adv_addr.addr[0]);
		is_update_beacon_info(temp_addr,param->rssi);
	}
	else
	{
		if(p3c_beacon_not_find_count < 200)//最大匹配次数
		{
			p3c_beacon_not_find_count++;//未匹配到蓝牙信标 次数累加
		}
		else
		{
			clear_beacon_info();//若200次都没扫描到则清空
		}
	}
	return KE_MSG_CONSUMED;
}



static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t find;
	UART_PRINTF("found %s operation = 0x%x,status = 0x%x,param->seq_num = 0x%x\r\n",__func__,param->operation,param->status,param->seq_num);
    switch(param->operation)
	{
	    case GATTC_SDP_DISC_SVC_ALL:
    	{
    		if(param->status == ATT_ERR_NO_ERROR)
    		{
    			ke_state_set(TASK_APP, APPM_CONNECTED);
                
                find = sdp_enable_all_server_ntf_ind(app_env.conidx,1);
                
                UART_PRINTF("find1= %d\r\n",find);	
    			
    		}
    	}
        break;
    	case GATTC_WRITE:
    	{
    		if(param->seq_num == 0xa5)
            {      
        		find = sdp_enable_all_server_ntf_ind(app_env.conidx,0);
        		
        	    ke_state_set(TASK_APP, APPM_CONNECTED);
        		
        		UART_PRINTF("find2= %d\r\n",find);
        		if(ke_timer_active(APP_START_SCAN_TIMER, TASK_APP))
                {
                    ke_timer_clear(APP_START_SCAN_TIMER, TASK_APP);
                }
                ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,300);
            }        	
    	}
        break;
        case GATTC_REGISTER:
        {
            ke_state_set(TASK_APP, APPM_CONNECTED);
        }
        break;
        case GATTC_MTU_EXCH:
        {
            //set_flash_clk(0x1);
#if (SMP_ENCRYPT_EN)        
            if(!app_sec_get_bond_status())
            {
                sdp_service_status_reset(app_env.conidx);
                ke_timer_set(APP_START_SMP_REQ_TIMER, TASK_APP,5);			
            }
            else
            {			
                ke_timer_set(APP_START_ENCRYPT_TIMER,TASK_APP,5);
            }
#else 
            ke_state_set(TASK_APP, APPM_SDP_DISCOVERING);
            sdp_discover_all_service(app_env.conidx);
#endif
        }
        break;
    }
	return (KE_MSG_CONSUMED);
}

static int gattc_sdp_svc_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_sdp_svc_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	uint8_t conidx = KE_IDX_GET(src_id);		
	sdp_extract_svc_info(conidx,ind);
	
	return (KE_MSG_CONSUMED);
}


/*******************************************************************************
 * Function: app_period_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *		  param -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance (TASK_GAP).
 *		  ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_period_timer_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    
   
	return KE_MSG_CONSUMED;

}

static int app_start_scan_timer_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    UART_PRINTF("app_start_scan_timer_handlerr,status=%x\n",ke_state_get(TASK_APP));
    if((ke_state_get(TASK_APP) != APPM_SCANNING )&&(ke_state_get(TASK_APP) != APPM_CONNECTING))
    {
        appm_start_scanning();
        ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,2000);
    }
    else if(ke_state_get(TASK_APP) == APPM_SCANNING )
    {
        appm_stop_scanning();
        
    }
    else
        ke_timer_set(APP_START_SCAN_TIMER,TASK_APP,100);
    
	return KE_MSG_CONSUMED;

}

extern struct rev_ntf_data notify_data;
static int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_event_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    UART_PRINTF("appm2 %s \r\n",__func__);
    //UART_PRINTF("type = 0x%x,length = 0x%x,handle = 0x%02x\r\n",param->type,param->length,param->handle);
    //UART_PRINTF("RECIVE value = \r\n");
    
    for(int i = 0; i< param->length; i++)
    {
        UART_PRINTF("%02x ",param->value[i]);
    }
    
    UART_PRINTF("%x ",param->value[0]);
    UART_PRINTF("\r\n");

    if(param->length==0x08)
    {
        notify_data.notify_standard_key_status=1;
        memcpy(&notify_data.notify_standard_key[1],param->value,param->length);

    }
    if(param->length==0x02)
    {
        notify_data.notify_media_key_status=1;
	 if((param->value[0] != 0) && (param->value[0] != 0))
	 {
        	memcpy(&notify_data.notify_media_key[1],param->value,param->length);
	 }
	  UART_PRINTF("%x  %x\r\n",notify_data.notify_media_key[1],notify_data.notify_media_key[2]);

    }
    if(param->length==0x01)
    {
        notify_data.notify_power_key_status=1;
        memcpy(&notify_data.notify_power_key[1],param->value,param->length);

    }
    if(param->length==0x04)
    {
        notify_data.notify_mouse_status=1;
        memcpy(&notify_data.notify_mouse[1],param->value,param->length);

    }
    if(param->length==0x13 )
    {
        notify_data.notify_voice_status=1;
	  decode_vocie_data(param->value,param->length);
      	  //  memcpy(&notify_data.notify_voice[notify_data.notify_voice_write*param->length],param->value,param->length);
        notify_data.notify_voice_write++;
        notify_data.notify_voice_write%=15;
			
    }
    if(param->length==0x12)
    {
        notify_data.notify_sensor_status=1;
        memcpy(&notify_data.notify_sensor[1],param->value,param->length);

    }
    test_usb_device();
    
    ///appm_write_uuid_data_req(0xfff2,param->length,(uint8_t *)&param->value[0]);
    return (KE_MSG_CONSUMED);
}

static int gattc_rd_rsp_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_read_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
	
    UART_PRINTF("read rsp:");	
    for(int i = 0; i< param->length; i++)
    {
        UART_PRINTF("%02x ",param->value[i]);
    }
    UART_PRINTF("\n");
	return (KE_MSG_CONSUMED);
}



static int app_sensor_get_val_handler(ke_msg_id_t const msgid,
										  struct gapm_profile_added_ind *param,
										  ke_task_id_t const dest_id,
										  ke_task_id_t const src_id)
{
	//UART_PRINTF("%s\r\n", __func__);
	sensor_cb_handler();
	timer_flag = 0;
	ke_timer_set(APP_SENSOR_GET_VAL_TIMER, TASK_APP, 1);
	return KE_MSG_CONSUMED;
}


  static int app_uart2_send_timer_handler(ke_msg_id_t const msgid,
								  struct gapc_param_update_req_ind const *param,
								  ke_task_id_t const dest_id,
								  ke_task_id_t const src_id)
	{
		char *temp_buf = NULL;
		char buf[UART_SEND_BUF_LEN_MAX] = {0};
		unsigned char *temp_mac=NULL;//记录当前信标mac
		//e.g. BKXYxxxx,UL03,UL100003|0|00002|0,UL2C0:30:50:44:F2:55,xxxx#
		
		uart_ul_data_t temp_u_data ={0};
		temp_u_data.step_num = get_steps();//获取步数
		temp_u_data.motion_state = get_move_state();//获取运动状态 0:静止 1.慢走 2.快走 3.慢跑 4.快跑
		temp_u_data.motion_interval = get_move_ticks_rec();//获取运动时长
		if(temp_u_data.motion_state)//处于运动状态
		{
			temp_u_data.motion_interval+=1;//+1秒
			set_move_ticks_rec(temp_u_data.motion_interval);//重新设置运动时长 单位：sec
		}
		temp_u_data.motion_sensor_state = get_motion_sensor_state();//存在与否 or 初始化是否成功
		temp_u_data.pressure_sensor_state = get_pressure_sensor_state();//存在与否or 初始化是否成功
		temp_mac = get_current_beacon_mac();//获取当前信标mac地址
		memcpy(temp_u_data.mac_str,temp_mac,strlen(temp_mac));
		//UART_PRINTF(">>>strlen(temp_mac):%d\r\n",strlen(temp_mac));
		temp_buf = uart_ul_data_joint(temp_u_data);
		memcpy(buf,temp_buf,strlen(temp_buf));
			
		//UART_PRINTF("%s>>>uart2 send :%s len:%d\r\n",__func__,buf,strlen(buf));
		uart2_send((unsigned char*)buf,strlen(buf));
		ke_timer_set(APP_UART2_TEST_TIMER,TASK_APP , 100);
		return KE_MSG_CONSUMED;
	}

/* Default State handlers definition. */
const struct ke_msg_handler appm_default_state[] =
{
	// Note: first message is latest message checked by kernel so default is put on top.
	{KE_MSG_DEFAULT_HANDLER,    	(ke_msg_func_t)appm_msg_handler},
	{GAPM_DEVICE_READY_IND,     	(ke_msg_func_t)gapm_device_ready_ind_handler},
	{GAPM_CMP_EVT,             		(ke_msg_func_t)gapm_cmp_evt_handler},
	{GAPC_GET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
	{GAPC_SET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
	{GAPC_CONNECTION_REQ_IND,   	(ke_msg_func_t)gapc_connection_req_ind_handler},
	{GAPC_CMP_EVT,             		(ke_msg_func_t)gapc_cmp_evt_handler},
	{GAPC_DISCONNECT_IND,       	(ke_msg_func_t)gapc_disconnect_ind_handler},
	{GAPM_PROFILE_ADDED_IND,    	(ke_msg_func_t)gapm_profile_added_ind_handler},
	{GAPC_LE_PKT_SIZE_IND,	       (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
	{GAPC_PARAM_UPDATED_IND,		(ke_msg_func_t)gapc_param_updated_ind_handler},
	{GATTC_MTU_CHANGED_IND,		(ke_msg_func_t)gattc_mtu_changed_ind_handler},
	{GAPC_PARAM_UPDATE_REQ_IND, 	(ke_msg_func_t)gapc_param_update_req_ind_handler},
	{APP_PARAM_UPDATE_REQ, 	(ke_msg_func_t)gapc_update_conn_param_req_handler},
	{GATTC_CMP_EVT,             	       (ke_msg_func_t)gattc_cmp_evt_handler},
	{GATTC_SDP_SVC_IND,         	       (ke_msg_func_t)gattc_sdp_svc_ind_handler},
	{GAPM_ADV_REPORT_IND,       	(ke_msg_func_t)gapm_adv_report_ind_handler},
	{APP_GET_RSSI_TIMER,			(ke_msg_func_t)app_get_rssi_timer_handler},
	{APP_GATTC_EXC_MTU_CMD,		(ke_msg_func_t)gattc_mtu_exchange_req_handler},
	{APP_START_SMP_REQ_TIMER,          (ke_msg_func_t)app_start_smpc_handler},
    {APP_START_ENCRYPT_TIMER,          (ke_msg_func_t)app_start_encrypt_handler},
	{GAPC_CON_RSSI_IND, 			(ke_msg_func_t)gapc_conn_rssi_ind_handler},
	{APP_DISCONNECT_TIMER,			(ke_msg_func_t)app_disconnect_timer_handler},
	{APP_PERIOD_TIMER,				(ke_msg_func_t)app_period_timer_handler},
	{APP_START_SCAN_TIMER,				(ke_msg_func_t)app_start_scan_timer_handler},
	{GATTC_EVENT_IND_1,               (ke_msg_func_t)gattc_event_ind_handler},
    {GATTC_RD_RSP_IND,(ke_msg_func_t)gattc_rd_rsp_event_ind_handler},
	
    {APP_SENSOR_GET_VAL_TIMER,		(ke_msg_func_t)app_sensor_get_val_handler},/*添加传感器timer和handler处理函数*/
    {APP_UART2_TEST_TIMER,			(ke_msg_func_t)app_uart2_send_timer_handler},/*添加uart2的timer和handler处理函数*/
};

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_default_state);

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
