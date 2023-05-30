#ifndef _HSCDHD003A_H_
#define _HSCDHD003A_H_

#include <stdint.h>

void hsactd003a_init(void);
void hsactd003a_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
void hsactd003a_read_accel_bytes(uint8_t *rawData);
void hsactd003a_readsteate_clear_pending(void);
void hsactd_set_to_sniff(void);
void hsactd_get_who_am_i(void);
//////void hsactd003a_set_to_cwake_read_start(void);
//////void hsactd003a_set_to_standby(void);
void  changeI2cAddress(unsigned char addr);
void delay_ms(unsigned int tt);
void delay_us(uint32_t num);
void hsppad042a_enable_meassurement(void);

#endif
