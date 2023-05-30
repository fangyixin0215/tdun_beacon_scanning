#include "rwip_config.h"             // SW configuration
#if (BLE_APP_PRESENT)
#include "app_task.h"                // Application task Definition
#include "app_sdp.h"              // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API
#include "gattc_task.h"
#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition
#include "sdp_service_task.h"
#include "sdp_service.h"
#include "uart.h"
#include "gpio.h"
#include "app_sec.h"
#include "flash.h"
#include "app.h"


/// Application Environment Structure
//struct useapp_env_tag useapp_env;
#if (BLE_CENTRAL || BLE_OBSERVER)
uint32_t con_peer_idx;
extern struct adv_addr_list_t adv_addr_list;
extern uint8_t max_scan_numbers;


void appm_set_max_scan_nums(uint8_t max)
{
	max_scan_numbers = max;
}

uint8_t appm_get_max_scan_nums(void)
{
	return max_scan_numbers;
}

void appm_start_scanning(void)
{
    UART_PRINTF("%s\r\n",__func__);
    if(appm_get_con_dev_num()>=APP_IDX_MAX)
    {
        UART_PRINTF("connect number max\r\n");
        return;

    }
    
    if((ke_state_get(TASK_APP) != APPM_SCANNING) )
    {
        // Prepare the GAPM_START_SCAN_CMD message
        struct gapm_start_scan_cmd *cmd = KE_MSG_ALLOC(GAPM_START_SCAN_CMD,
                                          TASK_GAPM, TASK_APP,
                                          gapm_start_scan_cmd);
        cmd->op.addr_src = GAPM_STATIC_ADDR;
        cmd->op.code = GAPM_SCAN_ACTIVE;  //GAPM_SCAN_PASSIVE; ///190726
        cmd->interval = 35;		//35;
        cmd->window  = 	35;	    //35;
        cmd->mode = GAP_OBSERVER_MODE;
        
#if WHITE_LIST_EN		
		if(app_sec_get_bond_status())
		{
			cmd->filt_policy = SCAN_ALLOW_ADV_WLST;
		}
		else
#endif			
		{
			cmd->filt_policy = SCAN_ALLOW_ADV_ALL;
		}
		
        cmd->filter_duplic = SCAN_FILT_DUPLIC_EN;
        adv_addr_list.nums = 0;
        // Send the message
        ke_msg_send(cmd);
        // Set the state of the task to APPM_SCANNING
        ke_state_set(TASK_APP, APPM_SCANNING);
    }
    else if(ke_state_get(TASK_APP) == APPM_SCANNING)
   	{
		//appm_stop_scanning();
		//ke_state_set(TASK_APP, APPM_RES_SCANNING);
		UART_PRINTF("APPM_SCANNING\r\n");
	}
	else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}

void appm_stop_scanning(void)
{
    UART_PRINTF("%s\r\n",__func__);
    if (ke_state_get(TASK_APP) == APPM_SCANNING)
    {
        // Prepare the GAPM_CANCEL_CMD message
        struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                      TASK_GAPM, TASK_APP,
                                      gapm_cancel_cmd);
        cmd->operation = GAPM_CANCEL;
        // Send the message
        ke_msg_send(cmd);
        // Go in ready state
        ke_state_set(TASK_APP, APPM_READY);
    }
    else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}


extern uint32_t con_peer_idx;
void appm_start_connencting(struct gap_bdaddr bdaddr)
{
    UART_PRINTF("%s\r\n",__func__);
    if (ke_state_get(TASK_APP) == APPM_SCANNING)
    {
        appm_stop_scanning();
        ke_state_set(TASK_APP, APPM_WAIT_SCANNEND);
        UART_PRINTF("APPM_WAIT_SCANNEND\r\n");
    }
    else if (ke_state_get(TASK_APP) == APPM_READY)
    {
        // Prepare the GAPM_START_SCAN_CMD message
        struct gapm_start_connection_cmd *cmd = KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_start_connection_cmd, sizeof(struct gap_bdaddr));
        cmd->op.addr_src = GAPM_STATIC_ADDR;
        cmd->op.code = GAPM_CONNECTION_DIRECT;//GAPM_CONNECTION_AUTO;//GAPM_CONNECTION_DIRECT;
        cmd->scan_interval = 10;
        cmd->scan_window  = 6;
        cmd->con_intv_min = 12;
        cmd->con_intv_max = 24;
        cmd->con_latency = 0;
        cmd->superv_to = 300;
        cmd->ce_len_min = 2;
        cmd->ce_len_max = 4;
		
        cmd->nb_peers = 1;
        cmd->peers[0].addr_type = bdaddr.addr_type;
        memcpy(&cmd->peers[0].addr.addr[0],&(bdaddr.addr.addr[0]),6);

		
        // Send the message
        ke_msg_send(cmd);
        // Set the state of the task to APPM_SCANNING
        ke_state_set(TASK_APP, APPM_CONNECTING);
    }
    //set_flash_clk(0xa);
}

void appm_recover_connencting(void)
{
    UART_PRINTF("%s\r\n",__func__);
    if (ke_state_get(TASK_APP) == APPM_READY)
    {
        // Prepare the GAPM_START_SCAN_CMD message
        struct gapm_start_connection_cmd *cmd = KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_start_connection_cmd, sizeof(struct gap_bdaddr));
        cmd->op.addr_src = GAPM_STATIC_ADDR;
        cmd->op.code = GAPM_CONNECTION_DIRECT;//GAPM_CONNECTION_GENERAL;
        cmd->scan_interval = 50;
        cmd->scan_window  = 40;
        cmd->con_intv_min = 16;
        cmd->con_intv_max = 20;
        cmd->con_latency = 0;
        cmd->superv_to = 80;
        cmd->ce_len_min = 100;
        cmd->ce_len_max = 200;
        cmd->nb_peers = 1;
        cmd->peers[0].addr_type = adv_addr_list.addr_type[con_peer_idx];
        memcpy(&cmd->peers[0].addr.addr[0],&(adv_addr_list.adv_addr[con_peer_idx]),6);
        UART_PRINTF("con_peer_idx = %d ,addr = 0x%02x%02x%02x%02x%02x%02x\r\n",con_peer_idx,\
                    cmd->peers[0].addr.addr[0],cmd->peers[0].addr.addr[1],cmd->peers[0].addr.addr[2],cmd->peers[0].addr.addr[3],cmd->peers[0].addr.addr[4],cmd->peers[0].addr.addr[5]);
        // Send the message
        ke_msg_send(cmd);
        // Set the state of the task to APPM_CONNECTING
        ke_state_set(TASK_APP, APPM_CONNECTING);
    }

}

void appm_stop_connencting(void)
{
    UART_PRINTF("%s\r\n",__func__);
    if (ke_state_get(TASK_APP) == APPM_CONNECTING)
    {
        // Go in ready state
        // Prepare the GAPM_CANCEL_CMD message
        struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                      TASK_GAPM, TASK_APP,
                                      gapm_cancel_cmd);
        cmd->operation = GAPM_CANCEL;
        // Send the message
        ke_msg_send(cmd);
    }
    // else ignore the request
    else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}

void appm_disconnect(void)
{
    UART_PRINTF("%s\r\n",__func__);
    if((ke_state_get(TASK_APP) == APPM_CONNECTED) ||(ke_state_get(TASK_APP) == APPM_LINK_CONNECTED)||(ke_state_get(TASK_APP) == APPM_SDP_DISCOVERING) )
    {
        struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                          KE_BUILD_ID(TASK_GAPC, 0), TASK_APP,
                                          gapc_disconnect_cmd);
        cmd->operation = GAPC_DISCONNECT;
        cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;
        // Send the message
        ke_msg_send(cmd);
    }
    else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}


uint8_t appm_adv_data_decode(uint8_t len,const uint8_t *data,uint8_t *find_str,uint8_t str_len)
{
    uint8_t find = 0;
    uint16_t index;
    for(index = 0; index < len;)
    {
        switch(data[index + 1])
        {
        case GAP_AD_TYPE_FLAGS:
        {
            //UART_PRINTF("AD_TYPE :%02x ",data[index + 2]);  
            index +=(data[index] + 1);
        }
        break;
        case GAP_AD_TYPE_SHORTENED_NAME:
        case GAP_AD_TYPE_COMPLETE_NAME:
        {
            if(strncmp((char*)&data[index + 2],(char*)find_str,str_len) == 0 )
            {
                find = 1;
            }
          /*  
            UART_PRINTF("ADV_NAME : ");
            for(uint8_t len = 0; len < data[index] - 1; len++)
            {
                UART_PRINTF("%c",data[index + 2 + len]);
            }
            UART_PRINTF("\r\n");
            */
            index +=(data[index] + 1);
        }
        break;
        case GAP_AD_TYPE_MORE_16_BIT_UUID:
        {/*
            UART_PRINTF("UUID : ");
            for(uint8_t len = 0; len < data[index] - 1;)
            {
                UART_PRINTF("%02x%02x  ",data[index + 2 + len],data[index + 3 + len]);
                len+=2;
            }
            UART_PRINTF("\r\n");
         */   
            index +=(data[index] + 1);
        }
        break;
        default:
        {
            index +=(data[index] + 1);
        }
        break;
        }
    }
    return find;
}

void appm_write_data_req(uint8_t conidx,uint16_t handle,uint8_t len,uint8_t *data)
{
    if(1)///ke_state_get(TASK_APP) == APPM_CONNECTED )
    {
        struct sdp_write_info_req *req = KE_MSG_ALLOC_DYN(SDP_WRITE_VALUE_INFO_REQ,
                                         prf_get_task_from_id(TASK_ID_SDP), TASK_APP,
                                         sdp_write_info_req, len);
        // Fill in the parameter structure
        req->handle = handle;
        req->length = len;
        req->operation=GATTC_WRITE_NO_RESPONSE;
        memcpy(req->data,data,len);
        // Send the message
        ke_msg_send(req);
    }
    else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}


void appc_write_service_ntf_cfg_req(uint8_t conidx,uint16_t handle,uint16_t ntf_cfg)
{
    UART_PRINTF("%s\r\n",__func__);

    UART_PRINTF("app_env.conidx= %x\r\n",app_env.conidx);
    if(1)//ke_state_get(TASK_APP) == APPM_CONNECTED )
    {
        struct sdp_write_ntf_cfg_req *req = KE_MSG_ALLOC(SDP_WRITE_NTF_CFG_REQ,
                                        prf_get_task_from_id(KE_BUILD_ID(TASK_ID_SDP,conidx)), KE_BUILD_ID(TASK_APP,conidx),
                                        sdp_write_ntf_cfg_req);
        UART_PRINTF("dest = 0x%04x\r\n",prf_get_task_from_id(KE_BUILD_ID(TASK_ID_SDP,conidx)));
        // Fill in the parameter structure
        req->conidx = conidx;
        req->handle = handle;
        req->ntf_cfg = ntf_cfg;
        // Send the message
        ke_msg_send(req);

        
    }
    else
    {
        UART_PRINTF("ke_state_get(TASK_APP) = %x\r\n",ke_state_get(TASK_APP));
    }
}
void appm_read_info_req(uint8_t conidx,uint16_t handle)
{
    UART_PRINTF("%s\r\n",__func__);
    
    struct sdp_read_info_req *req = KE_MSG_ALLOC(SDP_READ_INFO_REQ,
                                    prf_get_task_from_id(KE_BUILD_ID(TASK_ID_SDP,conidx)), KE_BUILD_ID(TASK_APP,conidx),
                                    sdp_read_info_req);
    
    // Fill in the parameter structure
    req->conidx = conidx;
    req->handle = handle;
    req->type= 0;
    // Send the message
    ke_msg_send(req);

    
  
}


uint8_t sdp_enable_all_server_ntf_ind(uint8_t conidx,uint8_t  reset)
{
    UART_PRINTF("func %s\r\n",__func__);
    bool more_enable = false;
    static uint8_t server_num = 0,chars_num = 0;
    if(reset == 1)
    {
        server_num = 0;
        chars_num = 0;
    }
    for(; server_num < SDP_NB_SERVICE_INSTANCES_MAX;)
    {
        UART_PRINTF("sdp_env_init.used_status[%x][%x]=%x\n",conidx,server_num,sdp_env_init.used_status[conidx][server_num]);
        ///if( (sdp_env_init.used_status[conidx][server_num] == USED_STATUS) )&& (sdp_env_init.sdp_env[conidx][server_num].conidx == conidx) )
        if( sdp_env_init.used_status[conidx][server_num] == USED_STATUS)
        {
            UART_PRINTF("server_num = %d,chars_nb = %d\r\n",server_num,sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_nb);
            for(; chars_num < sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_nb;)
            {
                UART_PRINTF("server_num = %d,chars_num = %d,prop = 0x%x\r\n",server_num,chars_num,sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].prop);
                if(sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].prop & ATT_CHAR_PROP_IND)
                {UART_PRINTF("-------------------\n");
                 
                    appc_write_service_ntf_cfg_req(conidx,sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].val_hdl,PRF_CLI_START_IND);
                    
                    more_enable = true;
                    chars_num++;
                    return more_enable;
                }
                else if(sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].prop & ATT_CHAR_PROP_NTF)
                {
                    
                    appc_write_service_ntf_cfg_req(conidx,sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].val_hdl,PRF_CLI_START_NTF);                   
                    more_enable = true;
                    chars_num++;
                    return more_enable;
                }
                else
                {
                    if(sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].prop & ATT_CHAR_PROP_RD)
                    {
                        appm_read_info_req(conidx,sdp_env_init.sdp_env[conidx][server_num].prf_db_env->sdp_cont->chars_descs_inf.chars_inf[chars_num].val_hdl);

                    }
                    chars_num++;
                }
            }
            server_num++;
            chars_num = 0;
        }
        else
        {
            more_enable = false;
            server_num++;
            //return more_enable;
        }
    }
    return more_enable;
}



#if 1
void sdp_prf_register_all_atthdl2gatt(uint8_t conidx)
{
    uint8_t i = 0;
    for(i = 0;i < SDP_NB_SERVICE_INSTANCES_MAX;i++)
    {
        struct sdp_env_tag* sdp_env = (struct sdp_env_tag*)&sdp_env_init.sdp_env[conidx][i];
//        struct prf_sdp_db_env *prf_db_env=&sdp_env_init.sdp_env[conidx][i].prf_db_env;
        UART_PRINTF("prf_register_atthdl2gatt start_hdl = 0x%x,end_hdl = 0x%x\r\n",sdp_env->prf_db_env->sdp_cont->svc.shdl,sdp_env->prf_db_env->sdp_cont->svc.ehdl);

        if((0 != sdp_env->prf_db_env->sdp_cont->descs_nb)&&(sdp_env_init.used_status[conidx][i]==USED_STATUS))         
        {	           
            UART_PRINTF("prf_register_atthdl2gatt start_hdl = 0x%x,end_hdl = 0x%x\r\n",sdp_env->prf_db_env->sdp_cont->svc.shdl,sdp_env->prf_db_env->sdp_cont->svc.ehdl);
            ///UART_PRINTF("prf_db_env->sdp_cont->svc.shdl = 0x%x,end_hdl = 0x%x\r\n",sdp_env->prf_db_env->sdp_cont->svc.shdl,sdp_env->prf_db_env->sdp_cont->svc.ehdl);
                

            prf_register_atthdl2gatt(&(sdp_env->prf_env),conidx, &sdp_env->prf_db_env->sdp_cont->svc);
        }
    }
}



#endif


#endif // (#if (BLE_CENTRAL || BLE_OBSERVER))
#endif  //#if (BLE_APP_PRESENT)

