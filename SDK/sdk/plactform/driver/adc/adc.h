

#ifndef _ADC_H_
#define _ADC_H_



#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions




void adc_init(uint8_t chanle,uint8_t mode);
void adc_isr(void);
void calib_adc(void);

#endif //



