
#include "ble_to_MTK_protocol.h"

#include "string.h"

//////////////////////////////////////////////////////////////////////////
p3c_ble_info_t p3c_ble_info[P3C_BLE_REC_NUM_MAX]={0};
unsigned char p3c_ble_rec_num = 0;//当前记录设备个数
unsigned short p3c_beacon_not_find_count = 0;//记录多少次扫描周期没找到需要的蓝牙信标设备了
//////////////////////////////////////////////////////////////////////////

void is_update_beacon_info(char * mac_addr,char rssi)
{
	unsigned char len = (unsigned char)strlen(mac_addr);
	if(strncmp((char*)&p3c_ble_info[0].addr,(char*)mac_addr,len) == 0)//这次信标跟上次一样
	{
		return;
	}
	else
	{
		memset(p3c_ble_info[0].addr,0,sizeof(p3c_ble_info[0].addr));
		memcpy(p3c_ble_info[0].addr,mac_addr,len);
	}
}
void clear_beacon_info(void)
{
	memset(p3c_ble_info[0].addr,0,sizeof(p3c_ble_info[0].addr));
	memcpy((char*)&p3c_ble_info[0].addr,"ff:ff:ff:ff:ff:ff",strlen("ff:ff:ff:ff:ff:ff"));
}

unsigned char * get_current_beacon_mac(void)
{
	return p3c_ble_info[0].addr;
}

