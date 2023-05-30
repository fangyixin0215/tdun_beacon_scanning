/**
 ****************************************************************************************
 *
 * @file ctss.h
 *
 * @brief Header file - CTS Service Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _CTSS_H_
#define _CTSS_H_

/**
 ****************************************************************************************
 * @addtogroup  CTS 'Profile' Server
 * @ingroup CTS
 * @brief CTS 'Profile' Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rwprf_config.h"

#if (BLE_CTS_SERVER)

#include "cts_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define CTSS_CFG_FLAG_MANDATORY_MASK       (0xFF)
#define CTS_CFG_FLAG_NTF_SUP_MASK         (0x08)
#define CTS_CFG_FLAG_MTP_FFF1_MASK         (0x40)


#define FFF1_FLAG_NTF_CFG_BIT             (0x02)




/// Battery Service Attributes Indexes
enum
{
	CTSS_IDX_SVC,

	CTSS_IDX_CUR_TIME_CHAR,
	CTSS_IDX_CUR_TIME_VAL,
	CTSS_IDX_CUR_TIME_NTF_CFG,
	
	CTSS_IDX_LOCAL_TIME_CHAR,
	CTSS_IDX_LOCAL_TIME_VAL,

	CTSS_IDX_REFER_TIME_CHAR,
	CTSS_IDX_REFER_TIME_VAL,
	
	CTSS_IDX_NB,
};

typedef int8_t tip_time_zone;

/**
 * DST Offset Characteristic - UUID:0x2A2D
 * Min value : 0, Max value : 8
 * 255 = DST is not known
 */
typedef uint8_t tip_dst_offset;
typedef uint8_t tip_time_source;

/**
 * Time Accuracy Characteristic - UUID:0x2A12
 * Accuracy (drift) of time information in steps of 1/8 of a second (125ms) compared
 * to a reference time source. Valid range from 0 to 253 (0s to 31.5s). A value of
 * 254 means Accuracy is out of range (> 31.5s). A value of 255 means Accuracy is
 * unknown.
 */
typedef uint8_t tip_time_accuracy;

struct tip_curr_time
{
    /// Date time
    struct prf_date_time date_time;
    /// Day of the week
    uint8_t day_of_week;
    /// 1/256th of a second
    uint8_t fraction_256;
    /// Adjust reason
    uint8_t adjust_reason;
};


struct tip_loc_time_info
{
    tip_time_zone time_zone;
    tip_dst_offset dst_offset;
};
struct tip_ref_time_info
{
    tip_time_source time_source;
    tip_time_accuracy time_accuracy;
    /**
     * Days since last update about Reference Source
     * Min value : 0, Max value : 254
     * 255 = 255 or more days
     */
    uint8_t days_update;
    /**
     * Hours since update about Reference Source
     * Min value : 0, Mac value : 23
     * 255 = 255 or more days (If Days Since Update = 255, then Hours Since Update shall
     * also be set to 255)
     */
    uint8_t hours_update;
};
 union tips_rd_value_tag
{
        ///Current Time
        struct tip_curr_time curr_time;
        ///Local Time Information
        struct tip_loc_time_info loc_time_info;
        ///Reference Time Information
        struct tip_ref_time_info ref_time_info;
 };

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// CTS 'Profile' Server environment variable
struct ctss_env_tag
{
    /// profile environment
    prf_env_t prf_env;
   
    /// On-going operation
    struct ke_msg * operation;
    /// CTS Services Start Handle
    uint16_t start_hdl;
    uint8_t local_time_value[CTS_LOCAL_TIME_DATA_LEN];
	
	uint8_t cur_time_value[CTS_CUR_TIME_DATA_LEN];
	uint8_t refer_time_value[CTS_REFER_TIME_DATA_LEN];
    /// BASS task state
    ke_state_t state[CTSS_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
    /// Database features
    uint8_t features;

};



/**
 ****************************************************************************************
 * @brief Retrieve fff0 service profile interface
 *
 * @return fff0 service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* ctss_prf_itf_get(void);

uint16_t ctss_get_att_handle(uint8_t att_idx);

uint8_t  ctss_get_att_idx(uint16_t handle, uint8_t *att_idx);

void ctss_notify_cur_time(struct ctss_env_tag* ctss_env, struct ctss_cur_time_upd_req const *param);

#endif /* #if (BLE_CTS_SERVER) */



#endif /*  _CTS_H_ */



