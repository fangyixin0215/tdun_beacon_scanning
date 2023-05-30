/**
 ****************************************************************************************
 *
 * @file   ctss_task.c
 *
 * @brief FFF0 Server Role Task Implementation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CTS_SERVER)

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "atts.h"
#include "co_utils.h"
#include "cts.h"
#include "cts_task.h"
#include "uart.h"
#include "prf_utils.h"


uint8_t tips_pack_curr_time_value(uint8_t *p_pckd_time, const struct tip_curr_time* p_curr_time_val)
{
    // Date-Time
    prf_pack_date_time(p_pckd_time, &(p_curr_time_val->date_time));

    //Day of Week
    *(p_pckd_time + 7) = p_curr_time_val->day_of_week;

    //Fraction 256
    *(p_pckd_time + 8) = p_curr_time_val->fraction_256;

    //Adjust Reason
    *(p_pckd_time + 9) = p_curr_time_val->adjust_reason;

    return 10;
}

static int ctss_cur_time_upd_req_handler(ke_msg_id_t const msgid,
                                            struct ctss_cur_time_upd_req const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_SAVED;
    uint8_t state = ke_state_get(dest_id);
	
    // check state of the task
    if(state == CTSS_IDLE)
    {
        struct ctss_env_tag* ctss_env = PRF_ENV_GET(CTSS, ctss);

        // put task in a busy state
        ke_state_set(dest_id, CTSS_BUSY);						
		ctss_notify_cur_time(ctss_env, param);
		ke_state_set(dest_id, CTSS_IDLE);   
		msg_status = KE_MSG_CONSUMED;	
    }

    return (msg_status);
  }


  
static int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_att_info_req_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{

    struct gattc_att_info_cfm * cfm;
    uint8_t  att_idx = 0;
    // retrieve handle information
    uint8_t status = ctss_get_att_idx(param->handle, &att_idx);

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;

    if(status == GAP_ERR_NO_ERROR)
    {
        // check if it's a client configuration char
        if(att_idx == CTSS_IDX_CUR_TIME_NTF_CFG)
        {
            // CCC attribute length = 2
            cfm->length = 2;
        }
        // not expected request
        else
        {
            cfm->length = 0;
            status = ATT_ERR_WRITE_NOT_PERMITTED;
        }
    }

    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}



static int gattc_write_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattc_write_cfm * cfm;
    uint8_t att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = ctss_get_att_idx(param->handle,  &att_idx);
			UART_PRINTF("%s\r\n", __func__);

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        struct ctss_env_tag* ctss_env = PRF_ENV_GET(CTSS, ctss);
        // Extract value before check
        uint16_t ntf_cfg = co_read16p(&param->value[0]);

        // Only update configuration if value for stop or notification enable
        if ((att_idx == CTSS_IDX_CUR_TIME_NTF_CFG)
                && ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_NTF)))
        {

            // Conserve information in environment
            if (ntf_cfg == PRF_CLI_START_NTF)
            {
            	UART_PRINTF("ntf enble\r\n");
                // Ntf cfg bit set to 1
                ctss_env->ntf_cfg[conidx] |= (CTS_CUR_TIME_NTF_SUP );
            }
            else
            {
            	UART_PRINTF("ntf disble\r\n");
                // Ntf cfg bit set to 0
                ctss_env->ntf_cfg[conidx] &= ~(CTS_CUR_TIME_NTF_SUP );
            }

            // Inform APP of configuration change
            struct ctss_cur_time_ntf_cfg_ind * ind = KE_MSG_ALLOC(CTSS_CUR_TIME_NTF_CFG_IND,
                    prf_dst_task_get(&(ctss_env->prf_env), conidx), dest_id,
                    ctss_cur_time_ntf_cfg_ind);
            ind->conidx = conidx;
            ind->ntf_cfg = ctss_env->ntf_cfg[conidx];
						
            ke_msg_send(ind);			
        }
		else
        {
            status = PRF_APP_ERROR;
        }

    }

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}   



static int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattc_read_cfm * cfm;
    uint8_t  att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = ctss_get_att_idx(param->handle, &att_idx);
    uint16_t length = 0;
    struct ctss_env_tag* ctss_env = PRF_ENV_GET(CTSS, ctss);
	 struct tip_curr_time curr_time;
        ///Local Time Information
     struct tip_loc_time_info loc_time_info;
        ///Reference Time Information
     struct tip_ref_time_info ref_time_info;

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        // read notification information
        if (att_idx == CTSS_IDX_CUR_TIME_VAL)
        {
            length = CTS_CUR_TIME_DATA_LEN * sizeof(uint8_t);
        }
        // read notification information
        else if (att_idx == CTSS_IDX_CUR_TIME_NTF_CFG)
        {
            length = sizeof(uint16_t);
        }
		else if (att_idx == CTSS_IDX_LOCAL_TIME_VAL)
        {
            length = sizeof(struct tip_loc_time_info);
        }
		else if (att_idx == CTSS_IDX_REFER_TIME_VAL)
        {
            length = sizeof(struct tip_ref_time_info);
        }
        else
        {
            status = PRF_APP_ERROR;
        }
    }

    //Send write response
    cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, length);
    cfm->handle = param->handle;
    cfm->status = status;
    cfm->length = length;
   
    if (status == GAP_ERR_NO_ERROR)
    {
        // read notification information
        if (att_idx == CTSS_IDX_CUR_TIME_VAL)
        {
        	UART_PRINTF("read cur time\r\n");
            cfm->length = tips_pack_curr_time_value(cfm->value, &curr_time);
        }
        // retrieve notification config
        else if (att_idx == CTSS_IDX_CUR_TIME_NTF_CFG)
        {
            uint16_t ntf_cfg = (ctss_env->ntf_cfg[conidx] & CTS_CUR_TIME_NTF_SUP) ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;
            co_write16p(cfm->value, ntf_cfg);
        }  
		else if (att_idx == CTSS_IDX_LOCAL_TIME_VAL)
        {
        	UART_PRINTF("read local time\r\n");
            memcpy(cfm->value, (uint8_t *)&loc_time_info, sizeof(loc_time_info));
        }
		else if (att_idx == CTSS_IDX_REFER_TIME_VAL)
        {
        	UART_PRINTF("read refer time\r\n");
            memcpy(cfm->value, (uint8_t *)&ref_time_info, sizeof(ref_time_info));
        }
        else
        {
            /* Not Possible */
        }
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}   


static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

	
    if(param->operation == GATTC_NOTIFY)
    {	
    
      	uint8_t conidx = KE_IDX_GET(src_id);
      	struct ctss_env_tag* ctss_env = PRF_ENV_GET(CTSS, ctss);

	    struct ctss_cur_time_upd_rsp *rsp = KE_MSG_ALLOC(CTSS_CUR_TIME_UPD_RSP,
                                                         prf_dst_task_get(&(ctss_env->prf_env), conidx),
                                                         dest_id, ctss_cur_time_upd_rsp);

        rsp->status = param->status;			
        ke_msg_send(rsp);
    }
	
	// go back in to idle mode
    ke_state_set(dest_id, ke_state_get(dest_id) & ~CTSS_BUSY);
	
    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler ctss_default_state[] =
{
    {CTSS_CUR_TIME_UPD_REQ,      (ke_msg_func_t) ctss_cur_time_upd_req_handler},
    {GATTC_ATT_INFO_REQ_IND,        (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND,           (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND,            (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t) gattc_cmp_evt_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler ctss_default_handler = KE_STATE_HANDLER(ctss_default_state);

#endif /* #if (BLE_FFF0_SERVER) */


