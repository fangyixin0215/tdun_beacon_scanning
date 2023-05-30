/**
 ****************************************************************************************
 *
 * @file ctss_task.h
 *
 * @brief Header file - Battery Service Server Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _CTSS_TASK_H_
#define _CTSS_TASK_H_


#include "rwprf_config.h"
#if (BLE_CTS_SERVER)
#include <stdint.h>
#include "rwip_task.h" // Task definitions
#include "prf_types.h"
/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of FFF0 Server task instances
#define CTSS_IDX_MAX     0x01
///Maximal number of FFF0 that can be added in the DB

#define  CTS_LOCAL_TIME_DATA_LEN  20
#define  CTS_REFER_TIME_DATA_LEN  20
#define  CTS_CUR_TIME_DATA_LEN  10

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the CTSS task
enum ctss_state
{
    /// Idle state
    CTSS_IDLE,
    /// busy state
    CTSS_BUSY,
    /// Number of defined states.
    CTSS_STATE_MAX
};

/// Messages for FFF0 Server
enum ctss_msg_id
{
    /// Start the FFF0 Server - at connection used to restore bond data
	CTSS_CREATE_DB_REQ   = TASK_FIRST_MSG(TASK_ID_CTSS),
	
    /// FFF1 Level Value Update Request
    CTSS_CUR_TIME_UPD_REQ,
    /// Inform APP if FFF1 Level value has been notified or not
    CTSS_CUR_TIME_UPD_RSP,
    /// Inform APP that FFF1 Level Notification Configuration has been changed - use to update bond data
    CTSS_CUR_TIME_NTF_CFG_IND,
			
};

/// Features Flag Masks
enum ctss_features
{
    /// FFF1 Level Characteristic doesn't support notifications
    CTS_CUR_TIME_NTF_NOT_SUP,
    /// FFF1 Level Characteristic support notifications
    CTS_CUR_TIME_NTF_SUP,
};
/*
 * APIs Structures
 ****************************************************************************************
 */

/// Parameters for the database creation
struct ctss_db_cfg
{
    /// Number of FFF0 to add
    uint8_t fff0_nb;
    /// Features of each FFF0 instance
    uint8_t features;
   };

/// Parameters of the @ref CTSS_ENABLE_REQ message
struct ctss_enable_req
{
    /// connection index
    uint8_t  conidx;
    /// Notification Configuration
    uint8_t  ntf_cfg;
    /// Old FFF1 Level used to decide if notification should be triggered
    uint8_t  old_fff1_lvl;
};


///Parameters of the @ref CTSS_BATT_LEVEL_UPD_REQ message
struct ctss_cur_time_upd_req
{
    /// BAS instance
    uint8_t conidx;
	
	uint8_t length;
	
    uint8_t cur_time_value[CTS_CUR_TIME_DATA_LEN];
};

struct ctss_cur_time_upd_rsp
{
    ///status
    uint8_t status;
};

///Parameters of the @ref BASS_BATT_LEVEL_NTF_CFG_IND message
struct ctss_cur_time_ntf_cfg_ind
{
    /// connection index
    uint8_t  conidx;
    ///Notification Configuration
    uint8_t  ntf_cfg;
};



/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler ctss_default_handler;
#endif // BLE_FFF0_SERVER


#endif /* _CTSS_TASK_H_ */

