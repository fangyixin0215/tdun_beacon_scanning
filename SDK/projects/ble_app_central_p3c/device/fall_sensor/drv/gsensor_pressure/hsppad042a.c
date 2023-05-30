#include "hsppad042a.h"
#include "i2c.h"
//#include "arch_console.h"


#define		HSPPAD042A_ADDRESS		0x48

#define		HSPPAD042A_WIA			0x00
#define		HSPPAD042A_INFO			0x01
#define		HSPPAD042A_FFST			0x02
#define		HSPPAD042A_STAT			0x03
#define		HSPPAD042A_POUTL		0x04
#define		HSPPAD042A_POUTM		0x05
#define		HSPPAD042A_POUTH		0x06
#define		HSPPAD042A_TOUTL		0x09
#define		HSPPAD042A_TOUTH		0x0A
#define		HSPPAD042A_DCTL			0x0D
#define		HSPPAD042A_CTL1			0x0E
#define		HSPPAD042A_CTL2			0x0F
#define		HSPPAD042A_ACTL1		0x10
#define		HSPPAD042A_ACTL2		0x11
#define		HSPPAD042A_FCTL			0x12
#define		HSPPAD042A_AVCL			0x13
#define		HSPPAD042A_I2CD			0x15
#define		HSPPAD042A_PNUM			0x1C
#define		HSPPAD042A_PDET			0x20
#define		HSPPAD042A_TDET			0x22
#define		HSPPAD042A_SRST			0x26
#define		HSPPAD042A_PTDET		0x29


//#define		HSPPAD042A_USE_IIR

#define arch_printf	uart_printf
int uart_printf(const char *fmt,...);
//unsigned char i2c_addr_003a = 0;
//unsigned char i2c_addr_042a = 0;
void writeOneByte_1(unsigned char x,unsigned char y)
{
	i2cs_tx_data(i2c_addr_042a,x,&y,1);
}

void readOneByte_1(unsigned char x,unsigned char *y)
{
	i2cs_rx_data(i2c_addr_042a,x,y,1);
}


static void hsppad042a_softReset(void)
{

	/*soft reset*/
	writeOneByte_1(HSPPAD042A_ACTL2,0x80);
	
//	delay_ms(3);  // 必须>2.5ms 	
}

void hsppad042a_checkID(void)
{
	uint8_t read_data = 0;
	changeI2cAddress(HSPPAD042A_ADDRESS);
	readOneByte_1(HSPPAD042A_WIA, &read_data);
	if(read_data == 0x49)
	{
		set_pressure_sensor_state(1);//启动成功 or 初始化成功
		//arch_printf("hsppad042a_checkID :0x%x \r\n",read_data);
	}
	else
	{
		set_pressure_sensor_state(0);//启动失败 or 初始化失败
		//arch_printf("hsppad042a_checkID :0x%x error!~\r\n",read_data);
	}

}

static void hsppad042a_setModefrequency(void)
{
//	writeOneByte_1(HSPPAD042A_CTL2,0x29);//100hz,only pressure measurement
	writeOneByte_1(HSPPAD042A_CTL2,0x25);//10hz,only pressure measurement
}
static void hsppad042a_setAverage(void)
{
	writeOneByte_1(HSPPAD042A_AVCL,0x04); //16average
}	

void hsppad042a_Init(void)
{

	changeI2cAddress(HSPPAD042A_ADDRESS);
	hsppad042a_checkID();
//	hsppad042a_softReset();
	
	writeOneByte_1(HSPPAD042A_CTL1,0x03);
	hsppad042a_setAverage();
	hsppad042a_setModefrequency();
	

//	writeOneByte_1(HSPPAD042A_CTL1,0x03);	
	
//	writeOneByte_1(HSPPAD042A_CTL2,0x02);	
//	writeOneByte_1(HSPPAD042A_AVCL,0x04);	
//	
//	writeActionCommand(HSPPAD042A_PDET);
}

#ifdef HSPPAD042A_USE_IIR
#define HSPPAD042A_IIR_W 2
#define HSPPAD042A_IIR_CO 16
static int alps_iir_filter(int raw_data)  
{
		static float  last_data	=	0;
    float filter_data;
    if(last_data != 0){
      filter_data = 1.0*(last_data*HSPPAD042A_IIR_CO
            +HSPPAD042A_IIR_W*raw_data-HSPPAD042A_IIR_W
            *last_data)/HSPPAD042A_IIR_CO;
    }else{
      filter_data = raw_data;
    }
    last_data = filter_data;

    return filter_data;
}
#endif

int hspaad042a_getdata(void)
{
	int pressure = 0;
	uint8_t read_data[3] = {0};
	
	changeI2cAddress(HSPPAD042A_ADDRESS);
	readOneByte_1(HSPPAD042A_POUTL, &read_data[0]);
	readOneByte_1(HSPPAD042A_POUTM, &read_data[1]);
	readOneByte_1(HSPPAD042A_POUTH, &read_data[2]);
	pressure =((((uint32_t)read_data[2])<<16) | (((uint32_t)read_data[1])<<8) | ((uint32_t)read_data[0])); 	
	
	return pressure;
}

void hspaad042a_read_rawdataBytes(uint8_t *rawData)
{
	changeI2cAddress(HSPPAD042A_ADDRESS);
	readOneByte_1(HSPPAD042A_POUTL, &rawData[0]);
	readOneByte_1(HSPPAD042A_POUTM, &rawData[1]);
	readOneByte_1(HSPPAD042A_POUTH, &rawData[2]);
	
}

void hsppad042a_standby(void)
{
	changeI2cAddress(HSPPAD042A_ADDRESS);
	writeOneByte_1(HSPPAD042A_CTL2,0x01);//1hz,continuous measurement mode,disable temperature and pressure measurement
}

void hsppad042a_enable_meassurement(void)
{
	changeI2cAddress(HSPPAD042A_ADDRESS);
	hsppad042a_setModefrequency();
	hsppad042a_setAverage();
}


