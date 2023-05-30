#ifndef __UART_DATA_H__
#define __UART_DATA_H__

#define UART_RECV_BUF_LEN_MAX 256//uart 接收最大长度
#define UART_SEND_BUF_LEN_MAX 256//uart 发送最大长度

#define UART_PACK_HEAD "BKXY"
#define UART_DATA_SEPARATOR_FLAG ","
#define UART_DATA_END_FLAG "#"
/**************************************************
*数据类型重定义
**************************************************/

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

typedef struct
{
	uint8_t motion_sensor_state;//跌倒传感器集成的三轴计工作状态 0：Err 1：OK
	uint8_t pressure_sensor_state;//跌倒传感器集成的气压计工作状态 0：Err 1：OK
	uint8_t motion_state;//运动状态
	uint8_t fall_state;//跌倒状态
	uint16_t step_num;//步数
	uint16_t motion_interval;//运动时长
	char mac_str[20];
}uart_ul_data_t;

/**************************************************
*函数名: crc16bitbybit
*函数声明
*param:
* uint8_t *ptr :传入的数据
* uint16_t len :数据长度
* ret : 校验结果
**************************************************/
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len);


/**************************************************
*函数名: uart_ul_data_joint
*函数声明
*param:
* uart_ul_data_t com_data :传入的数据
* ret : 拼接数据结果
**************************************************/
char * uart_ul_data_joint(uart_ul_data_t com_data);

#endif
