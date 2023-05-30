/**
 ****************************************************************************************
 *
 * @file app_sec.c
 *
 * @brief Application Security Entry Point
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

#include "rwip_config.h"

#if (BLE_APP_SEC)

#include <string.h>
#include "co_math.h"
#include "gapc_task.h"      // GAP Controller Task API Definition
#include "gap.h"            // GAP Definition
#include "gapc.h"           // GAPC Definition
#include "prf_types.h"
#include "llc_llcp.h"

#include "app.h"            // Application API Definition
#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definitionde 
#include "ke_timer.h"

#if (DISPLAY_SUPPORT)
#include "app_display.h"    // Display Application Definitions
#endif //(DISPLAY_SUPPORT)

#if (NVDS_SUPPORT)
#include "nvds.h"           // NVDS API Definitions
#endif //(NVDS_SUPPORT)

#ifdef BLE_APP_AM0
#include "am0_app.h"
#endif // BLE_APP_AM0
#include "co_utils.h"
#include "uart.h"
#include "sdp_service.h"
#include "user_config.h"
#include "app.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init()
{
#if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_PERIPH_BONDED;
    if (nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
    {
    // If read value is invalid, set status to not bonded
        if ((app_sec_env.bonded != true) && (app_sec_env.bonded != false))
        {
            app_sec_env.bonded = false;
        }
    }
    UART_PRINTF("app_sec_env.bonded=%x\n",app_sec_env.bonded);
#endif	
}

bool app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}



#if  (NVDS_SUPPORT )
void app_sec_remove_bond(uint8_t index)
{
    // Check if we are well bonded
/*    
    if (app_sec_env[index].bonded == true)
    {
        // Update the environment variable
        app_sec_env[index].bonded = false;

        if (nvds_put(NVDS_TAG_PERIPH_BONDED+index, NVDS_LEN_PERIPH_BONDED,
                     (uint8_t *)&app_sec_env[index].bonded) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
    }
	app_sec_env[index].bonded = false;
*/	
}
#endif 


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(ke_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	UART_PRINTF("%s,param->request=%x \r\n",__func__,param->request);
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KE_MSG_ALLOC(GAPC_BOND_CFM,
                                             src_id, TASK_APP,
                                             gapc_bond_cfm);
    //uint8_t conidx = KE_IDX_GET(src_id);
    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
	    	UART_PRINTF("gapc_pairing req\r\n");
            cfm->request = GAPC_PAIRING_RSP;
            cfm->accept  = false;

            // Check if we are already bonded (Only one bonded connection is supported)
            if (!app_sec_env.bonded)
            {
                cfm->accept  = true;

                #if (BLE_APP_HID || BLE_APP_HT)
                // Pairing Features
				cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
                #elif defined(BLE_APP_AM0)
                cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
                #else
                cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
                #endif 

                #if (BLE_APP_HT)
                cfm->data.pairing_feat.iocap     = GAP_IO_CAP_DISPLAY_ONLY;
                #else
                cfm->data.pairing_feat.iocap     = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
                #endif //(BLE_APP_HT)

                cfm->data.pairing_feat.key_size  = 16;
                cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
                cfm->data.pairing_feat.sec_req   = GAP_NO_SEC;
                cfm->data.pairing_feat.ikey_dist = GAP_KDIST_ENCKEY| GAP_KDIST_IDKEY;
                cfm->data.pairing_feat.rkey_dist = GAP_KDIST_ENCKEY| GAP_KDIST_IDKEY;   
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
			UART_PRINTF("gapc_ltk req\r\n");
            // Counter
            uint8_t counter;
            cfm->accept  = true;
            cfm->request = GAPC_LTK_EXCH;

            // Generate all the values
            cfm->data.ltk.ediv = (uint16_t)co_rand_word();

            for (counter = 0; counter < RAND_NB_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
                cfm->data.ltk.randnb.nb[counter] = (uint8_t)co_rand_word();
                //UART_PRINTF("ltk.key=%x\r\n",cfm->data.ltk.ltk.key[counter]);
            }

            for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
                //UART_PRINTF("ltk.key=%x\r\n",cfm->data.ltk.ltk.key[counter]);
            }
#if 0//(NVDS_SUPPORT)
            // Store the generated value in NVDS
            if (nvds_put(NVDS_TAG_LTK+conidx, NVDS_LEN_LTK,(uint8_t *)&cfm->data.ltk) != NVDS_OK)
            {
                ASSERT_ERR(0);
            }
#endif// #if (NVDS_SUPPORT)
        } break;


        case (GAPC_IRK_EXCH):
        {
			UART_PRINTF("gapc_lrk exch\r\n");

            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key, app_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;
            memcpy(cfm->data.irk.addr.addr.addr,(uint8_t *)&co_default_bdaddr,BD_ADDR_LEN);
        } break;


        case (GAPC_TK_EXCH):
        {
            // Generate a PIN Code- (Between 100000 and 999999)
			#if(PINCODE_SET) 
			uint32_t pin_code = PIN_CODE_VALUE;
			#else
			uint32_t pin_code = (100000 + (co_rand_word()%900000));
			#endif
            UART_PRINTF("pin_code=%d\r\n",pin_code);
            cfm->accept  = true;
            cfm->request = GAPC_TK_EXCH;

            // Set the TK value
            memset(cfm->data.tk.key, 0, KEY_LEN);

            cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
            cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
            cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
            cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
        } break;

		case (GAPC_CSRK_EXCH): 
		{
			cfm->accept  = true;
        	cfm->request = GAPC_CSRK_EXCH;
		}	
		break;
		
        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


static int gapc_bond_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    UART_PRINTF("%s param->info = 0x%x\r\n",__func__,param->info);
    uint8_t conidx = KE_IDX_GET(src_id);
    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
			// Update the bonding status in the environment

            //app_sec_env[conidx].bonded = true;
	    	UART_PRINTF("gapc pairing success\r\n");
            // Update the bonding status in the environment
#if 0//(NVDS_SUPPORT)
            
            if (nvds_put(NVDS_TAG_PERIPH_BONDED+conidx, NVDS_LEN_PERIPH_BONDED,
                         (uint8_t *)&app_sec_env[conidx].bonded) != NVDS_OK)
            {
                // An error has occurred during access to the NVDS
                ASSERT_ERR(0);
            }
            
            // Set the BD Address of the peer device in NVDS
            if (nvds_put(NVDS_TAG_PEER_BD_ADDRESS+conidx, NVDS_LEN_PEER_BD_ADDRESS,
                         (uint8_t *)gapc_get_bdaddr(conidx, SMPC_INFO_PEER)) != NVDS_OK)
            {
                // An error has occurred during access to the NVDS
                ASSERT_ERR(0);
            }
#endif //(PLF_NVDS)

			struct gap_bdaddr *peer_addr = gapc_get_bdaddr(conidx, SMPC_INFO_PEER);
			UART_PRINTF("peer addr: \r\n");
			UART_PRINTF("addr.type = %x\r\n",peer_addr->addr_type);
			UART_PRINTF("addr.addr = ");
			for(int i = 0;i<sizeof(struct bd_addr);i++)
			{
				UART_PRINTF("0x%x ",peer_addr->addr.addr[i]);
			}
			UART_PRINTF("\r\n");
        } break;

        case (GAPC_REPEATED_ATTEMPT):
        {
            appm_disconnect();
        } break;

        case (GAPC_IRK_EXCH):
        {
		   	UART_PRINTF("gapc pairing GAPC_IRK_EXCH\r\n");
           	#if 0//(NVDS_SUPPORT)
           	// Store peer identity in NVDS
           	if (nvds_put(NVDS_TAG_PEER_IRK, NVDS_LEN_PEER_IRK, (uint8_t *)&param->data.irk) != NVDS_OK)
           	{
               	ASSERT_ERR(0);
           	}
           	#endif // (NVDS_SUPPORT)
					 
			memcpy(&app_env.peer_irk,&param->data.irk.irk.key[0],sizeof(struct gapc_irk));

            UART_PRINTF("irk.key = ");
			for(int i = 0;i<sizeof(struct gap_sec_key);i++)
			{
				UART_PRINTF("0x%x ",param->data.irk.irk.key[i]);
			}
			UART_PRINTF("\r\n");
					
			UART_PRINTF("addr.type = %x\r\n",param->data.irk.addr.addr_type);
			UART_PRINTF("addr.addr = ");
			for(int i = 0;i<sizeof(struct bd_addr);i++)
			{
				UART_PRINTF("0x%x ",param->data.irk.addr.addr.addr[i]);
			}
			UART_PRINTF("\r\n");
			
        } break;

		//配对的时候主机发起配对请求，从设备会生成LTK，主机这边需要保存下LTK，在主机发起加密请求的时候会用到
		case (GAPC_LTK_EXCH):  
		{ 
            uint8_t ltk_buff[36];
#if (NVDS_SUPPORT)            
           	// Store peer identity in NVDS
           	memcpy(ltk_buff,gapc_get_bdaddr(conidx, SMPC_INFO_PEER),6);
            memcpy(&ltk_buff[6],(uint8_t *)&param->data.ltk,NVDS_LEN_LTK2);
            
           	if (nvds_put(NVDS_TAG_LTK2+conidx, NVDS_LEN_LTK2, ltk_buff) != NVDS_OK)
           	{
               	ASSERT_ERR(0);
                UART_PRINTF("~~~~~~~~~~~~~~*************~~~~~~~~~");
           	}
#endif // (NVDS_SUPPORT)
            
			UART_PRINTF("data.ltk.ediv = 0x%x\r\n", param->data.ltk.ediv);
			UART_PRINTF("data.ltk.ltk.key:");
            for (uint8_t i = 0; i < RAND_NB_LEN; i++)
            {
				UART_PRINTF("0x%x ", param->data.ltk.ltk.key[i]);
            }
			UART_PRINTF("\r\n");
			UART_PRINTF("data.ltk.randnb.nb:");
			for (uint8_t i = 0; i < RAND_NB_LEN; i++)
            {
				UART_PRINTF("0x%x ", param->data.ltk.randnb.nb[i]);
            }
			UART_PRINTF("\r\n");

			
		}break;	

        case (GAPC_PAIRING_FAILED):
        {
	    	UART_PRINTF("gapc pairing failed\r\n");
	    	appm_disconnect();
        } break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
	if(ke_timer_active(APP_SEND_SECURITY_REQ, TASK_APP))
	{
		ke_timer_clear(APP_SEND_SECURITY_REQ, TASK_APP);
	}
	
	UART_PRINTF("%s \r\n",__func__);
    #if (NVDS_SUPPORT)
    // LTK value
    struct gapc_ltk ltk;
    // Length
    uint8_t length = NVDS_LEN_LTK;
    #endif 
    uint8_t conidx = KE_IDX_GET(src_id);
    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KE_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                                src_id, KE_BUILD_ID(TASK_APP,conidx),
                                                gapc_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.bonded)  
    {
        #if (NVDS_SUPPORT)
        // Retrieve the required informations from NVDS
        if (nvds_get(NVDS_TAG_LTK+conidx, &length, (uint8_t *)&ltk) == NVDS_OK)
        {
            // Check if the provided EDIV and Rand Nb values match with the stored values
            if ((param->ediv == ltk.ediv) &&
                !memcmp(&param->rand_nb.nb[0], &ltk.randnb.nb[0], sizeof(struct rand_nb)))
            {
                cfm->found    = true;
                cfm->key_size = 16;
                memcpy(&cfm->ltk, &ltk.ltk, sizeof(struct gap_sec_key));
            }
            /*
             * else we are bonded with another device, disconnect the link
             */
        }
        else
        {
        	UART_PRINTF("read ltk fail\r\n");

        }
        #endif // #if (NVDS_SUPPORT)
		
		// Send the message
    	ke_msg_send(cfm);
    }
	else
	{
		//TCL BOX
		cfm->found    = false;
		cfm->key_size = 16;
		memset(&cfm->ltk, 0x0, sizeof(struct gap_sec_key));
		
		UART_PRINTF("bond lost,set flag\r\n");

		ke_msg_send(cfm);
		appm_disconnect();
	}

    return (KE_MSG_CONSUMED);
}


static int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded
	UART_PRINTF("%s \r\n",__func__);
	uint8_t conidx = KE_IDX_GET(src_id);
	if(param->status == 0x0)
	{
		UART_PRINTF("encrypt success, auth = %d\r\n", param->auth);
	}
	else
	{
		UART_PRINTF("encrypt failed, clr bond info\r\n");
		app_sec_remove_bond(conidx);
	}


    return (KE_MSG_CONSUMED);
}

static int app_sec_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message

    return (KE_MSG_CONSUMED);
}

void app_sec_bond_cmd_req(void)
{
	  struct gapc_bond_cmd *cmd = KE_MSG_ALLOC(GAPC_BOND_CMD,
            			KE_BUILD_ID(TASK_GAPC, app_env.conidx), KE_BUILD_ID(TASK_APP, app_env.conidx),//TASK_APP,//KE_BUILD_ID(TASK_APP, app_env.conidx),
            			gapc_bond_cmd);

	UART_PRINTF("%s\r\n", __func__);
    cmd->operation = GAPC_BOND;
    cmd->pairing.iocap = 0x04;
    cmd->pairing.oob = 0x00;
	cmd->pairing.auth = 0x05;
    cmd->pairing.key_size = 16;
    cmd->pairing.ikey_dist = 0x07;
    cmd->pairing.rkey_dist = 0x07;

	ke_msg_send(cmd);
}

void app_sec_encry_cmd_req(void)
{

	UART_PRINTF("send encryped start\r\n");
	extern uint8_t ltk_buffer[36];
    struct gapc_ltk ltk;
    // Length
    uint8_t length = NVDS_LEN_LTK2;
	if (nvds_get(NVDS_TAG_LTK2+app_env.conidx, &length,(uint8_t *)&ltk) != NVDS_OK)
    {
        ASSERT_ERR(0);
    }
    memcpy(&ltk,&ltk_buffer[6],NVDS_LEN_LTK2);
#if 1	
	UART_PRINTF("data.ltk.ediv = 0x%x\r\n", ltk.ediv);
	UART_PRINTF("data.ltk.ltk.key:");
    for (uint8_t i = 0; i < RAND_NB_LEN; i++)
    {
		UART_PRINTF("0x%x ", ltk.ltk.key[i]);
    }
	UART_PRINTF("\r\n");
	UART_PRINTF("data.ltk.randnb.nb:");
	for (uint8_t i = 0; i < RAND_NB_LEN; i++)
    {
		UART_PRINTF("0x%x ", ltk.randnb.nb[i]);
    }
	UART_PRINTF("\r\n");
#endif
	struct gapc_encrypt_cmd *cmd = KE_MSG_ALLOC(GAPC_ENCRYPT_CMD,
                        KE_BUILD_ID(TASK_GAPC, app_env.conidx),KE_BUILD_ID(TASK_APP, app_env.conidx),//TASK_APP,  
                        gapc_encrypt_cmd);

	cmd->operation = GAPC_ENCRYPT;
	cmd->ltk = ltk;
	
	ke_msg_send(cmd);
}


/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)app_sec_msg_dflt_handler},
    {GAPC_BOND_REQ_IND,       (ke_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND,           (ke_msg_func_t)gapc_bond_ind_handler},
    {GAPC_ENCRYPT_REQ_IND,    (ke_msg_func_t)gapc_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND,        (ke_msg_func_t)gapc_encrypt_ind_handler},
};

const struct ke_state_handler app_sec_table_handler =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct ke_msg_handler))};

#endif //(BLE_APP_SEC)

/// @} APP
