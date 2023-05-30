#ifndef __HSPPAD042A__
#define __HSPPAD042A__

#include <stdint.h>


extern unsigned char i2c_addr_003a;
extern unsigned char i2c_addr_042a;

void hsppad042a_Init(void);
void hsppad042a_enable_meassurement(void);
int hspaad042a_getdata(void);
void hsppad042a_standby(void);
void hsppad042a_checkID(void);

void hspaad042a_read_rawdataBytes(uint8_t *rawData);
extern void  changeI2cAddress(unsigned char addr);
extern void delay_ms(unsigned int tt);

void writeOneByte_1(unsigned char x,unsigned char y);
void readOneByte_1(unsigned char x,unsigned char *y);

#endif
