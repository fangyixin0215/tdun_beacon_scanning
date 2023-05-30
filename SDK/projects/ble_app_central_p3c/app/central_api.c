#include <string.h>
#include "uart.h"
#include "gpio.h"
#include "app_sdp.h"
#include "central_api.h"


uint8_t char_to_num(uint8_t ch);
uint8_t char_to_hex(uint8_t *ch);
extern unsigned char uart_rx_buf[];
extern uint32_t con_peer_idx;



void interface_data_pro(uint8_t *buf,uint8_t len)
{
}

uint8_t char_to_num(uint8_t ch)
{
    uint8_t value;
    if( (ch>='0')&&(ch<='9'))
    {
        value=(ch-'0');
    }
    else if( (ch>='a')&&(ch<='f'))
    {
        value=(ch-'a'+10);
    }
    else if ( (ch>='A')&&(ch<='F'))
    {
        value=(ch-'A'+10);
    }
    return value;
}

uint8_t char_to_hex(uint8_t *ch)
{
    uint8_t value;
    value = (char_to_num(ch[0])<< 4) | (char_to_num(ch[1]));
    return value;
}




