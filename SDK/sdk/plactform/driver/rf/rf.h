/**
 ****************************************************************************************
 *
 * @file rf.h
 *
 * @brief Common header file for all radios.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef RF_H_
#define RF_H_

/**
 ****************************************************************************************
 * @addtogroup RF
 * @ingroup DRIVERS
 * @brief Common definitions for radio modules.
 *
 * This module declares the functions and constants that have to be defined for all RF.
 *
 * @{
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

struct rwip_rf_api;  // forward declaration to avoid including rw.h

/**
 *****************************************************************************************
 * @brief Initialization of RF.
 *
 * This function initializes the RF and fills the structure containing the function
 * pointers and parameters required by the RW BT stack.
 *
 * @param[out]  api  Pointer to the BT RF API structure
 *
 *****************************************************************************************
 */
void rf_init(struct rwip_rf_api *api);


void initial_xver_BK3435V2_openLoop(void);


void Delay(int num);

void Delay_ms(int num);

void Delay_us(int num);

void mcu_use_48m(void);

void ldo_mode_set(void);
void buck_mode_set(void);

/// @} RF

#endif // RF_H_
