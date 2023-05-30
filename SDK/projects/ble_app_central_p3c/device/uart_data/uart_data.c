

#include "uart_data.h"
const uint16_t polynom = 0xA001;

/**************************************************
*函数名：crc16bitbybit
*函数定义
*param:
* uint8_t *ptr :传入的数据
* uint16_t len :数据长度
* polynom :多项式
* ret: 返回值->校验结果
**************************************************/
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;

	if (len == 0) {
	  len = 1;
	}
	while (len--) {
	  crc ^= *ptr;
	  for (i = 0; i<8; i++)
	  {
	   if (crc & 1) {
	    crc >>= 1;
	    crc ^= polynom;
	   }
	   else {
	    crc >>= 1;
	   }
	  }
	  ptr++;
	}
	return(crc);
}


/**************************************************
*函数名: uart_ul_data_joint
*函数定义
*param:
* uart_ul_data_t com_data :传入的数据
* ret : 返回值->拼接数据结果
**************************************************/

char * uart_ul_data_joint(uart_ul_data_t com_data)
	
{
	char temp_buf[UART_RECV_BUF_LEN_MAX+1] = {0};
	unsigned short crc16_calc_ret = 0;
	//               BKXYxxxx,UL03,UL100003|0|00002|0,UL2C0:30:50:44:F2:55,xxxx#
	
	unsigned short len = (unsigned short)strlen(",UL03,UL100003|0|00002|0,UL2C0:30:50:44:F2:55,xxxx#");//此案例中长度是固定的

	//第一次拼接 不带CRC校验
	sprintf(temp_buf,"BKXY%04d,UL0%01d,UL1%05d|%01d|%05d|%01d,UL2%s",len,
		(com_data.motion_sensor_state|((com_data.pressure_sensor_state)<<1)),
		com_data.step_num,com_data.motion_state,com_data.motion_interval,com_data.fall_state,
		com_data.mac_str);

	//获取当前temp_buf的crc16校验结果
	crc16_calc_ret = crc16bitbybit((uint8_t *)temp_buf+9,strlen(temp_buf));

	//清空temp_buf
	memset(temp_buf,0,sizeof(temp_buf));

	//第二次拼接 带CRC校验
	sprintf(temp_buf,"BKXY%04d,UL0%01d,UL1%05d|%01d|%05d|%01d,UL2%s,%04x#",len,
		(com_data.motion_sensor_state|((com_data.pressure_sensor_state)<<1)),
		com_data.step_num,com_data.motion_state,com_data.motion_interval,com_data.fall_state,
		com_data.mac_str,
		crc16_calc_ret
		);

	return (char *)temp_buf;
}

