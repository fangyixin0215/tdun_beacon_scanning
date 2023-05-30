/**
 ****************************************************************************************
 *
 * @file ctss.c
 *
 * @brief cts Server Implementation.
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CTS_SERVER)
#include "attm.h"
#include "cts.h"
#include "cts_task.h"
#include "prf_utils.h"
#include "prf.h"
#include "ke_mem.h"

#include "uart.h"



/*
 * CTS ATTRIBUTES DEFINITION
 ****************************************************************************************
 */
 
/// Full CTS Database Description - Used to add attributes into the database
const struct attm_desc cts_att_db[CTSS_IDX_NB] =
{
    // CTS Service Declaration
    [CTSS_IDX_SVC]            =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},

	[CTSS_IDX_CUR_TIME_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    //  Characteristic Value
    [CTSS_IDX_CUR_TIME_VAL]   =   {ATT_CHAR_CT_TIME,PERM(RD, ENABLE), PERM(RI, ENABLE), CTS_CUR_TIME_DATA_LEN *sizeof(uint8_t)},
	[CTSS_IDX_CUR_TIME_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE)|PERM(WRITE_REQ, ENABLE), 0, 0},

	[CTSS_IDX_LOCAL_TIME_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [CTSS_IDX_LOCAL_TIME_VAL]   =   {ATT_CHAR_LOCAL_TIME_INFO, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(struct tip_loc_time_info)},

	[CTSS_IDX_REFER_TIME_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [CTSS_IDX_REFER_TIME_VAL]   =   {ATT_CHAR_REFERENCE_TIME_INFO, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(struct tip_ref_time_info)},

};/// Macro used to retrieve permission value from access and rights on attribute.


static uint8_t ctss_init (struct prf_task_env* env, uint16_t* start_hdl, uint16_t app_task, uint8_t sec_lvl,  struct ctss_db_cfg* params)
{
    uint16_t shdl;
    struct ctss_env_tag* ctss_env = NULL;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;
    	 	UART_PRINTF("ctss_init------------------------------------\r\n");

    //-------------------- allocate memory required for the profile  ---------------------
    ctss_env = (struct ctss_env_tag* ) ke_malloc(sizeof(struct ctss_env_tag), KE_MEM_ATT_DB);
    memset(ctss_env, 0 , sizeof(struct ctss_env_tag));

    // Service content flag
    uint8_t cfg_flag = CTSS_CFG_FLAG_MANDATORY_MASK;

    // Save database configuration
    ctss_env->features |= (params->features) ;
   
    // Check if notifications are supported
    if (params->features == CTSS_IDX_CUR_TIME_NTF_CFG)
    {
        cfg_flag |= CTS_CFG_FLAG_NTF_SUP_MASK;
    }
    shdl = *start_hdl;

    //Create CTS in the DB
    //------------------ create the attribute database for the profile -------------------
    status = attm_svc_create_db(&(shdl), ATT_SVC_CURRENT_TIME, (uint8_t *)&cfg_flag,
            CTSS_IDX_NB, NULL, env->task, &cts_att_db[0],
            (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)));
				


    //Set optional permissions
    if (status == GAP_ERR_NO_ERROR)
    {
        //Set optional permissions
        if(params->features == CTS_CUR_TIME_NTF_SUP)
        {
            // Battery Level characteristic value permissions
            uint16_t perm = PERM(RD, ENABLE) | PERM(NTF, ENABLE);

            attm_att_set_permission(shdl + CTSS_IDX_CUR_TIME_VAL, perm, 0);
        }
    }

    //-------------------- Update profile task information  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {

        // allocate BASS required environment variable
        env->env = (prf_env_t*) ctss_env;
        *start_hdl = shdl;
        ctss_env->start_hdl = *start_hdl;
        ctss_env->prf_env.app_task = app_task
                | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        ctss_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_CTSS;
        env->desc.idx_max           = CTSS_IDX_MAX;
        env->desc.state             = ctss_env->state;
        env->desc.default_handler   = &ctss_default_handler;

        // service is ready, go into an Idle state
        ke_state_set(env->task, CTSS_IDLE);
    }
    else if(ctss_env != NULL)
    {
        ke_free(ctss_env);
    }
     
    return (status);
}


static void ctss_destroy(struct prf_task_env* env)
{
    struct ctss_env_tag* ctss_env = (struct ctss_env_tag*) env->env;

    // clear on-going operation
    if(ctss_env->operation != NULL)
    {
        ke_free(ctss_env->operation);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(ctss_env);
}

static void ctss_create(struct prf_task_env* env, uint8_t conidx)
{
    struct ctss_env_tag* ctss_env = (struct ctss_env_tag*) env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    ctss_env->ntf_cfg[conidx] = 0;
}


static void ctss_cleanup(struct prf_task_env* env, uint8_t conidx, uint8_t reason)
{
    struct ctss_env_tag* ctss_env = (struct ctss_env_tag*) env->env;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
    // force notification config to zero when peer device is disconnected
    ctss_env->ntf_cfg[conidx] = 0;
}


void ctss_notify_cur_time(struct ctss_env_tag* ctss_env, struct ctss_cur_time_upd_req const *param)
{
    // Allocate the GATT notification message
    struct gattc_send_evt_cmd *cur_time = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
            KE_BUILD_ID(TASK_GATTC, 0), prf_src_task_get(&(ctss_env->prf_env),0),
            gattc_send_evt_cmd, sizeof(uint8_t)* (param->length));

    // Fill in the parameter structure
    cur_time->operation = GATTC_NOTIFY;
    cur_time->handle = ctss_get_att_handle(CTSS_IDX_CUR_TIME_VAL);
    // pack measured value in database
    cur_time->length = param->length;
  	//fff1_lvl->value[0] = ctss_env->fff1_lvl[0];
	memcpy(&cur_time->value[0],&param->cur_time_value[0],param->length);
    // send notification to peer device
    ke_msg_send(cur_time);
}



/// BASS Task interface required by profile manager
const struct prf_task_cbs ctss_itf =
{
        (prf_init_fnct) ctss_init,
        ctss_destroy,
        ctss_create,
        ctss_cleanup,
};


const struct prf_task_cbs* ctss_prf_itf_get(void)
{
   return &ctss_itf;
}


uint16_t ctss_get_att_handle( uint8_t att_idx)
{
		
    struct ctss_env_tag *ctss_env = PRF_ENV_GET(CTSS, ctss);
    uint16_t handle = ATT_INVALID_HDL;
   
    handle = ctss_env->start_hdl;

    // increment index according to expected index
    if(att_idx < CTSS_IDX_NB)
    {
        handle += att_idx;
    }		      
    else
    {
        handle = ATT_INVALID_HDL;
    }
    

    return handle;
}

uint8_t ctss_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct ctss_env_tag* ctss_env = PRF_ENV_GET(CTSS, ctss);
    uint16_t hdl_cursor = ctss_env->start_hdl;
    uint8_t status = PRF_APP_ERROR;

    // Browse list of services
    // handle must be greater than current index 
    // check if it's a mandatory index
    if(handle <= (hdl_cursor + CTSS_IDX_REFER_TIME_VAL))
    {
        *att_idx = handle -hdl_cursor;
        status = GAP_ERR_NO_ERROR;
        
    }
    
    return (status);
}


#endif // (BLE_fff0_SERVER)


 
