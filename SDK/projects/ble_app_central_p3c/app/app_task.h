/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief Header file - APPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_TASK_H_
#define APP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * RWBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>         	// Standard Integer
#include "rwip.h"
#include "rwip_task.h"      	// Task definitions
#include "ke_task.h"        	// Kernel Task



/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of APP Task Instances
#define APP_IDX_MAX                 (BLE_CONNECTION_MAX)


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of APP task
enum appm_state
{
    /// Initialization state
    APPM_INIT,
    /// Database create state
    APPM_CREATE_DB,
    /// Ready State
    APPM_READY,

    /// Scanning state
    APPM_SCANNING,

    APPM_WAIT_SCANNEND,

	APPM_RES_SCANNING,
	
    /// Connecting state
    APPM_CONNECTING,
    
    /// Link Connected state
    APPM_LINK_CONNECTED,

    APPM_SDP_DISCOVERING,

    APPM_CONNECTED,//Prf Connected

    APPM_DISCONNECT,

    APPM_STATE_MAX
};


/// APP Task messages
enum appm_msg
{
    APPM_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),
	APP_PARAM_UPDATE_REQ,
	APP_SEND_SECURITY_REQ,
	APP_GATTC_EXC_MTU_CMD,
	APP_GET_RSSI_TIMER,
	APP_START_ENCRYPT_TIMER,
	APP_START_SMP_REQ_TIMER,
	APP_DISCONNECT_TIMER,
	APP_PERIOD_TIMER,
	APP_START_SCAN_TIMER,
	
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler appm_default_handler;
extern ke_state_t appm_state[APP_IDX_MAX];

/// @} APPTASK

#endif //(BLE_APP_PRESENT)

#endif // APP_TASK_H_
