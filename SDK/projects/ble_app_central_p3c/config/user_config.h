/**
 
*******************************************************************************
 *
 * @file user_config.h
 *
 * @brief Application configuration definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 
*******************************************************************************
 */ 
#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_


//�����Ҫʹ��GPIO���е��ԣ���Ҫ�������
#define GPIO_DBG_MSG			0
//UARTʹ�ܿ��ƺ�
#define UART_PRINTF_EN			1
//����Ӳ�����Կ���
#define DEBUG_HW				0


#define  SMP_ENCRYPT_EN                       1              ///SMP���ܹ���
#define  PINCODE_SET                          0              ///PINCODE ����
#define  PIN_CODE_VALUE                       (123456)
#define  ADDR_CHANGE_EN	                      0              ///������ַ����
#define  APP_GET_RSSI_EN                      1              ///���ӻ�ȡRSSI 
#define  APP_DIR_READ_NOTIFY                  0              ///ֱ��read rsp,receive notify data

//DRIVER CONFIG
#define UART_DRIVER                     1
#define GPIO_DRIVER                     0
#define AUDIO_DRIVER                    0
#define RTC_DRIVER                      0
#define ADC_DRIVER                      0
#define I2C_DRIVER                      0
#define PWM_DRIVER                      0
#define USB_DRIVER                      1 
#endif



