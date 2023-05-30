/**
 ****************************************************************************************
 *
 * @file app_sdp.h
 *
 * @brief Application entry point
 *
 * Copyright (C) Beken 2009-2016
 *
 *
 ****************************************************************************************
 */
#ifndef APP_SDP_H_
#define APP_SDP_H_
#include "rwip_config.h"     // SW configuration
#if (BLE_APP_PRESENT)
#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>           // Common BT Definitions
#include "arch.h"            // Platform Definitions
#include "gap.h"
#include "gapc.h"            // GAPC Definitions
#include "gattc_task.h"
#include <stdbool.h>
#include "ke_task.h"
#include "co_error.h"
#include "attm.h"
#include "gattc_task.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "sdp_service.h"
#if (BLE_CENTRAL || BLE_OBSERVER)
void appm_start_scanning(void);
void appm_stop_scanning(void);
uint8_t appm_adv_data_decode(uint8_t len,const uint8_t *data,uint8_t *find_str,uint8_t str_len);
void appm_start_connencting(struct gap_bdaddr bdaddr);
void appm_recover_connencting(void);
void appm_creat_connenct(void);
void appm_write_data_req(uint8_t conidx,uint16_t handle,uint8_t len,uint8_t *data);
void appm_write_ntf_cfg_req(uint16_t uuid,uint16_t char_num,uint16_t ntf_cfg);
void appm_read_uuid_data_req(uint16_t uuid);
void appm_read_uuid_cfg_req(uint16_t uuid);
void appm_read_uuid_user_desc_req(uint16_t uuid);
uint8_t  sdp_enable_all_server_ntf_ind(uint8_t conidx,uint8_t  reset);
void sdp_prf_register_all_atthdl2gatt(uint8_t conidx);
void uart_data_pro(void);
void appm_set_max_scan_nums(uint8_t max);
uint8_t appm_get_max_scan_nums(void);

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void appm_disconnect(void);
struct adv_addr_list_t
{
    uint8_t nums;
    uint8_t addr_type[30];
    struct bd_addr adv_addr[30];
};
#endif //#if (BLE_CENTRAL || BLE_OBSERVER)
#endif
#endif
