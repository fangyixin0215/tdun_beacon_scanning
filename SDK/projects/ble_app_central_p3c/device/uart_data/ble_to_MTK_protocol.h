
#ifndef __BLE_TO_MTK_PROTOCOL_H__
#define __BLE_TO_MTK_PROTOCOL_H__
#include "uart.h"
#define P3C_BLE_REC_NUM_MAX 5//列表记录最大的蓝牙mac地址数量

typedef struct
{
	unsigned char addr[20];
	unsigned char rssi;
}p3c_ble_info_t;


extern p3c_ble_info_t p3c_ble_info[P3C_BLE_REC_NUM_MAX];
extern unsigned char p3c_ble_rec_num;//当前记录设备个数
extern unsigned short p3c_beacon_not_find_count;//记录多少次扫描周期没找到需要的蓝牙信标设备了

//函数声明
void is_update_beacon_info(char * mac_addr,char rssi);
void clear_beacon_info(void);
unsigned char * get_current_beacon_mac(void);
#endif

