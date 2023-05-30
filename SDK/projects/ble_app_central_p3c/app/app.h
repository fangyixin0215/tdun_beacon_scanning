/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_H_
#define APP_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Application entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>           // Common BT Definitions
#include "arch.h"            // Platform Definitions
#include "gapc.h"            // GAPC Definitions
#include "user_config.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)

#if USB_DRIVER
#include "sbc_decoder.h"
#endif

/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (18)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if (NVDS_SUPPORT)
/// List of Application NVDS TAG identifiers
enum app_nvds_tag
{
    /// BLE Application Advertising data
    NVDS_TAG_APP_BLE_ADV_DATA           = 0x0B,
    NVDS_LEN_APP_BLE_ADV_DATA           = 32,

    /// BLE Application Scan response data
    NVDS_TAG_APP_BLE_SCAN_RESP_DATA     = 0x0C,
    NVDS_LEN_APP_BLE_SCAN_RESP_DATA     = 32,

    /// Mouse Sample Rate
    NVDS_TAG_MOUSE_SAMPLE_RATE          = 0x38,
    NVDS_LEN_MOUSE_SAMPLE_RATE          = 1,
    /// Peripheral Bonded
    NVDS_TAG_PERIPH_BONDED              = 0x39,
    NVDS_LEN_PERIPH_BONDED              = 1,
    
    /// Peer Device BD Address
    NVDS_TAG_PEER_BD_ADDRESS            = 0x39+6,
    NVDS_LEN_PEER_BD_ADDRESS            = 7,
    /// Mouse Energy Safe
    
    NVDS_TAG_LTK                        = 0x39+6+6,
    NVDS_LEN_LTK                        = 28,
    /// PAIRING

	NVDS_TAG_LTK2                        = 0x39+6+6+6,
    NVDS_LEN_LTK2                        = 34,

};

enum app_loc_nvds_tag
{
    /// Audio mode 0 task
    NVDS_TAG_AM0_FIRST                  = NVDS_TAG_APP_SPECIFIC_FIRST, // 0x90
    NVDS_TAG_AM0_LAST                   = NVDS_TAG_APP_SPECIFIC_FIRST+16, // 0xa0

    /// Local device Identity resolving key
    NVDS_TAG_LOC_IRK,
    /// Peer device Resolving identity key (+identity address)
    NVDS_TAG_PEER_IRK,

    /// size of local identity resolving key
    NVDS_LEN_LOC_IRK                    = KEY_LEN,
    /// size of Peer device identity resolving key (+identity address)
    NVDS_LEN_PEER_IRK                   = sizeof(struct gapc_irk),
};
#endif // (NVDS_SUPPORT)


/// Application environment structure
struct app_env_tag
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;

    /// Last initialized profile
    uint8_t next_svc;

    /// Bonding status
    bool bonded;

    /// Device Name length
    uint8_t dev_name_len;
    /// Device Name
    uint8_t dev_name[APP_DEVICE_NAME_MAX_LEN];

    /// Local device IRK
    uint8_t loc_irk[KEY_LEN];
	
	/// Peer device IRK
    struct gapc_irk peer_irk;
	
};

/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Application environment
extern struct app_env_tag app_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the BLE demo application.
 ****************************************************************************************
 */
void appm_init(void);


/**
 ****************************************************************************************
 * @brief Initialize dir_read_notify application.
 ****************************************************************************************
 */
void app_dir_read_notify_init(void);


/**
 ****************************************************************************************
 * @brief Add a required service in the database
 ****************************************************************************************
 */
bool appm_add_svc(void);
/**
 ****************************************************************************************
 * @brief Send to request to update the connection parameters
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void appm_disconnect(void);
/**
 ****************************************************************************************
 * @brief Retrieve device name
 *
 * @param[out] device name
 *
 * @return name length
 ****************************************************************************************
 */
uint8_t appm_get_dev_name(uint8_t* name);


/**
 ****************************************************************************************
 * @brief Return if the device is currently bonded
 ****************************************************************************************
 */
//bool app_sec_get_bond_status(void);

void appm_get_conn_rssi(uint8_t conidx);
uint8_t appm_get_con_dev_num(void);



#if USB_DRIVER
extern SbcDecoderContext sbc_decoder;

uint8_t store_decode_data(uint8_t *buf,uint8_t store_num);

uint8_t  read_decode_data(uint8_t *buf);


uint8_t decode_vocie_data(const uint8_t* in_data,uint8_t in_len);
#endif

/// @} APP

#endif //(BLE_APP_PRESENT)

#endif // APP_H_
