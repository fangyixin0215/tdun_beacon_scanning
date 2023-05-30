/* Includes ------------------------------------------------------------------*/
#include "hscdhd003a.h"
#include "i2c.h"
//#include "delay.h"
//#include "arch_console.h"
#include "timer.h"
#include "i2c_software_drv.c"
/*---------------------------------------------------------------------------
 *	Register Address
 *---------------------------------------------------------------------------*/
#define		EXT_STAT_1		0x00			/*	R   : Extended Status 1 */
#define		EXT_STAT_2		0x01			/*	R   : Extended Status 2 */
#define		XOUT_LSB		0x02			/*	R   : XOUT_LSB */
#define		XOUT_MSB		0x03			/*	R   : XOUT_MSB */
#define		YOUT_LSB		0x04			/*	R   : YOUT_LSB */
#define		YOUT_MSB		0x05			/*	R   : YOUT_MSB */
#define		ZOUT_LSB		0x06			/*	R   : ZOUT_LSB */
#define		ZOUT_MSB		0x07			/*	R   : ZOUT_MSB */
#define		STATUS_1		0x08			/*	R   : Status 1 */
#define		STATUS_2		0x09			/*	R   : Status 2 */
#define		FREG_1			0x0D			/*	R/W : Feature 1 */
#define		FREG_2			0x0E			/*	R/W : Feature 2 */
#define		INIT_1			0x0F			/*	W   : Initialization 1 */
#define		MODE_C			0x10			/*	R/W : Mode Control */
#define		RATE_1			0x11			/*	R/W : Rate 1 */
#define		SNIFF_C			0x12			/*	R/W : Sniff Control */
#define		SNIFFTH_C		0x13			/*	R/W : Sniff threshold Control */
#define		SNIFFCF_C		0x14			/*	R/W : Sniff Configuration */
#define		RANGE_C			0x15			/*	R/W : Range Resolution Control */
#define		FIFO_C			0x16			/*	R/W : FIFO Control */
#define		INTR_C			0x17			/*	R/W : Interrupt Control */
#define		INIT_3			0x1A			/*	W   : Initialization 3 */
#define		SCRATCH			0x1B			/*	R/W : Scratchpad */
#define		PMCR			0x1C			/*	R/W : Power Mode Control */
#define		DMX				0x20			/*	R/W : Drive Motion X */
#define		DMY				0x21			/*	R/W : Drive Motion Y */
#define		DMZ				0x22			/*	R/W : Drive Motion Z */
#define		RESET			0x24			/*	W   : Reset */
#define		INIT_2			0x28			/*	W   : Initialization 2 */
#define		TRIGC			0x29			/*	R/W : Trigger Count */
#define		XOFFL			0x2A			/*	R/W : X-offset LSB */
#define		XOFFH			0x2B			/*	R/W : X-offset MSB */
#define		YOFFL			0x2C			/*	R/W : Y-offset LSB */
#define		YOFFH			0x2D			/*	R/W : Y-offset MSB */
#define		ZOFFL			0x2E			/*	R/W : Z-offset LSB */
#define		ZOFFH			0x2F			/*	R/W : Z-offset MSB */
#define		XGAIN			0x30			/*	R/W : X Gain */
#define		YGAIN			0x31			/*	R/W : Y Gain */
#define		ZGAIN			0x32			/*	R/W : Z Gain */


/*---------------------------------------------------------------------------
 *	Register Bit Definitions
 *----------------------------------------------------------------------------*/
#define REG11_ODR_100_or_80_HZ			0x08	/* ODR set in register-0x11,when Ultra-Low Power=100Hz,Precision =80Hz */
#define REG12_SNIFF_RATE		0x00	/* SNIFF rate set in register-0x12 */
#define REG13_SNIFF_THRS		0x84	/* SNIFF threshold set in register-0x13 */

#define REG1C_SPM_CSPM			0x44	/* Power mode set in register-0x1C CWAKE Power Mode:Precision Mode,Sniff Power Mode:Precision Mode*/
//#define REG1C_SPM_CSPM			0x43	/* Power mode set in register-0x1C modify to 0x44 by 2019-1-25*/

#define REG15_RNG_2G_RES_12BIT	0x04	/* range and resolution set in register-0x15 */
#define REG15_RNG_16G_RES_12BIT	0x34	/* range and resolution set in register-0x15 */
#define REG15_RNG_12G_RES_12BIT	0x44	/* range and resolution set in register-0x15 */
#define REG15_RNG_16G_RES_14BIT	0x35	/* range and resolution set in register-0x15 */

#define sniff_threshold_mg			(60)    //unit mg,  1g=512LSB
uint8 set_reg13_sniff_thresX = REG13_SNIFF_THRS;
uint8 set_reg13_sniff_thresY = REG13_SNIFF_THRS;
uint8 set_reg13_sniff_thresZ = REG13_SNIFF_THRS;

#define HSACTD003A_ADDRESS             (0x4c)			//pin10->GND  4c
//#define HSACTD003A_ADDRESS             (0x6C)		//pin10->VDD

#define arch_printf	uart_printf
int uart_printf(const char *fmt,...);
unsigned char i2c_addr_003a = 0;
unsigned char i2c_addr_042a = 0;

static void setSniffThresValue(void);
void  changeI2cAddress(unsigned char addr)
{
	//addr << 1;
	if(addr == HSACTD003A_ADDRESS)
		i2c_addr_003a = addr;
	else
		i2c_addr_042a = addr;
		
}


void delay_ms(unsigned int tt)
{
    unsigned int i,j;
    while(tt--)
    {
        for (j=0; j<1000; j++)
        {
            for (i=0; i<12; i++)
            {
                ;
            }
        }
    }
}

void delay_us(uint32_t num)
{

    uint32_t i,j;
    for(i=0; i<num; i++)
        for(j=0; j<3; j++)
        {
            ;
        }
}


void writeOneByte(unsigned char x,unsigned char y)
{

	i2cs_tx_data(i2c_addr_003a,x,&y,1);
}

void readOneByte(unsigned char x,unsigned char *y)
{
	i2cs_rx_data(i2c_addr_003a,x,y,1);
}

void hsactd003a_set_to_standby(void)
{
	writeOneByte(MODE_C,0x01);	
	
	delay_ms(3);		//must more than 2ms
}

void hsactd_get_who_am_i(void)
{
	uint8_t sx = 0;
	changeI2cAddress(HSACTD003A_ADDRESS);
//	hsactd003a_set_to_standby();
	writeOneByte(SCRATCH, 0x49);

	readOneByte(SCRATCH,&sx);
	if(sx!=0x49)
	{
		set_motion_sensor_state(0);//启动失败 or 初始化失败
		//uart_printf("hsactd003A_get_who_am_i error %d\r\n",sx);
	}
	else
	{
		set_motion_sensor_state(1);//启动成功 or 初始化成功
		//uart_printf("hsactd003A WIA:0x%02X\r\n", sx);
	}
}

static void hsactd003a_reset(void)
{
	/*------ Go to Stand-by ------*/
	hsactd003a_set_to_standby();
	
	/*------ Soft Reset ------*/
	writeOneByte(0x24,0x40);
	delay_ms(3);
	/*------ Set I2C ------*/
	writeOneByte(FREG_1, 0x40);
	delay_us(15);
	/*------ Initialize1 ------*/
	writeOneByte(INIT_1, 0x42);
	delay_us(15);
	
	/*------ Initialize2 ------*/
	writeOneByte(DMX,0x01);
	delay_us(15);
	
	/*------ Initialize3 ------*/
	writeOneByte(DMY, 0x80);
	delay_us(15);
	
	/*------ Initialize4 ------*/
	writeOneByte(INIT_2, 0x00);
	delay_us(15);
	
	/*------ Initialize5 ------*/
	writeOneByte(INIT_3, 0x00);
	delay_us(15);
}

static void hsactd003a_set_accel_config(void)
{
	/*------ Set OUTCFG ------*/
	writeOneByte(RANGE_C, REG15_RNG_16G_RES_14BIT);
	delay_us(15);
	
	/*------ Set FIFO_C, Reset ------*/
	writeOneByte(FIFO_C, 0x80);
	delay_us(15);
	
	/*------ Disable FIFO ------*/
	writeOneByte(FIFO_C, 0x00);
	delay_us(15);
	
	/*------ Set Rate ------*/
	writeOneByte(RATE_1,REG11_ODR_100_or_80_HZ);
//	HSACTD003A_WriteOneByte(RATE_1, 0x07);
	delay_us(15);
}

static void hsactd003a_set_sniff_config(void)
{
	/*------ Sniff Confingration, Reset ------*/
	writeOneByte(SNIFFCF_C, 0x80);
	delay_ms(5);
	
	setSniffThresValue();
	
	/*------ Sniff Set Rate ------*/
	writeOneByte(SNIFF_C, REG12_SNIFF_RATE);
	delay_us(15);
	
	/*------ Sniff Confingration, X_Thres -1 ------*/
	writeOneByte(SNIFFCF_C, 0x01);
	delay_us(15);
	/*------ Sniff Confingration, X_Thres -2 ------*/
	writeOneByte(SNIFFTH_C, set_reg13_sniff_thresX);
	delay_us(15);
	
	/*------ Sniff Confingration, Y_Thres -1 ------*/
	writeOneByte(SNIFFCF_C, 0x02);
	delay_us(15);
	/*------ Sniff Confingration, Y_Thres -2 ------*/
	writeOneByte(SNIFFTH_C, set_reg13_sniff_thresY);
	delay_us(15);
	
	/*------ Sniff Confingration, Z_Thres -1 ------*/
	writeOneByte(SNIFFCF_C, 0x03);
	delay_us(15);
	/*------ Sniff Confingration, Z_Thres -2 ------*/
	writeOneByte(SNIFFTH_C, set_reg13_sniff_thresZ);
	delay_us(15);
}

static void hsactd003a_enable_sniff_intr(void)
{
	writeOneByte(INTR_C, 0x07);
	delay_us(15);
}

static void hsactd003a_set_power_on(void)
{
	writeOneByte(PMCR, REG1C_SPM_CSPM);
	delay_us(15);
}

//void hsactd003a_read_gain(unsigned short *gain_x,unsigned short *gain_y,unsigned short *gain_z)
//{
//	uint8_t data[9]={0};

//	readOneByte(HSACTD003A_ADDRESS,XOFFL,9,data);

//	*gain_x = ((unsigned short)(data[1] >> 7) << 8) | data[6];
//	*gain_y = ((unsigned short)(data[3] >> 7) << 8) | data[7];
//	*gain_z = ((unsigned short)(data[5] >> 7) << 8) | data[8];
//}

void hsactd003a_set_to_cwake_read_start(void)
{
	hsactd003a_set_to_standby();
	/*------ Go to CWAKE ------*/
	writeOneByte(MODE_C, 0x05);
	
	hsactd003a_readsteate_clear_pending();
}

void hsactd_set_to_sniff(void)
{
	changeI2cAddress(HSACTD003A_ADDRESS);
	hsactd003a_set_to_standby();
	writeOneByte(MODE_C, 0x02);
}

void hsactd003a_readsteate_clear_pending(void)
{
	uint8_t sx = 0;
	
	changeI2cAddress(HSACTD003A_ADDRESS);
	/*Read status and clear INTN pin*/
	readOneByte(STATUS_2,&sx);
}

void hsactd003a_init(void)
{
	//uart_printf("%s......\r\n",__func__);

	changeI2cAddress(HSACTD003A_ADDRESS);
	hsactd003a_reset(); 
	hsactd003a_set_accel_config(); //Disable FIFO,Set range:16g,14bit, odr
	hsactd003a_set_sniff_config();		//Set sniff config
	hsactd003a_enable_sniff_intr();   //Enable sniff interrupt
	hsactd003a_set_power_on();			//Set Power on
	hsactd_get_who_am_i();
	hsactd003a_set_to_cwake_read_start();
//	hsactd_set_to_sniff();
}



void hsactd003a_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t sx[6]={0};
	
	changeI2cAddress(HSACTD003A_ADDRESS);
	readOneByte(XOUT_LSB,&sx[0]);
	readOneByte(XOUT_MSB,&sx[1]);
	readOneByte(YOUT_LSB,&sx[2]);
	readOneByte(YOUT_MSB,&sx[3]);
	readOneByte(ZOUT_LSB,&sx[4]);
	readOneByte(ZOUT_MSB,&sx[5]);
	
	//uart_printf("%x,%x,%x,%x,%x,%x",(int16_t)((((uint16_t)sx[1])<<8)	|	sx[0]),((((uint16_t)sx[1])<<8)	|	sx[0]),(uint16_t)sx[3],(uint16_t)sx[4],(uint16_t)sx[5]);
	*ax	=	(int16_t)((((uint16_t)sx[1])<<8)	|	sx[0]);
	*ay	=	(int16_t)((((uint16_t)sx[3])<<8)	|	sx[2]);
	*az	=	(int16_t)((((uint16_t)sx[5])<<8)	|	sx[4]);
	//uart_printf("x %x y %x z %x",*ax,*ay,*az);
}

void hsactd003a_read_accel_bytes(uint8_t *rawData)
{
	changeI2cAddress(HSACTD003A_ADDRESS);
	readOneByte(XOUT_LSB,&rawData[0]);
	readOneByte(XOUT_MSB,&rawData[1]);
	readOneByte(YOUT_LSB,&rawData[2]);
	readOneByte(YOUT_MSB,&rawData[3]);
	readOneByte(ZOUT_LSB,&rawData[4]);
	readOneByte(ZOUT_MSB,&rawData[5]);

}

void hsactd003a_read_gain(uint32_t *gain_x,uint32_t *gain_y,uint32_t *gain_z)
{
	uint8_t data[6]={0};
	
	readOneByte(XOFFH,&data[0]);
	readOneByte(XGAIN,&data[1]);
	
	readOneByte(YOFFH,&data[2]);
	readOneByte(YGAIN,&data[3]);
	
	readOneByte(ZOFFH,&data[4]);
	readOneByte(ZGAIN,&data[5]);

	*gain_x = (uint32_t)(((uint16_t)(data[0] >> 7) << 8) | data[1]);
	*gain_y = (uint32_t)(((uint16_t)(data[2] >> 7) << 8) | data[3]);
	*gain_z = (uint32_t)(((uint16_t)(data[4] >> 7) << 8) | data[5]);
}

static void setSniffThresValue(void)
{
	uint32_t gx=0,gy=0,gz=0;
	uint8_t Xval=0,Yval=0,Zval=0;
	
	hsactd003a_read_gain(&gx,&gy,&gz);
	
	Xval = (sniff_threshold_mg*(625*gx+25000))>>19;  //2^19=544000
	if(Xval<4){
		Xval = 4;
	}
	if(Xval>=63){
		Xval = 62;
	}
	
	Yval = (sniff_threshold_mg*(625*gy+25000))>>19;  //2^19=544000
	if(Yval<4){
		Yval = 4;
	}
	if(Yval>=63){
		Yval = 62;
	}
	
	Zval = (sniff_threshold_mg*(625*gz+25000))>>19;  //2^19=544000
	if(Zval<4){
		Zval = 4;
	}
	if(Zval>=63){
		Zval = 62;
	}
	
	set_reg13_sniff_thresX = 0x80 | Xval; 
	set_reg13_sniff_thresY = 0x80 | Yval; 
	set_reg13_sniff_thresZ = 0x80 | Zval; 
	
	//arch_printf("gxyz= %d %d %d set_reg13_sniff_thresXYZ = 0x%x 0x%x 0x%x\r\n",gx,gy,gz,set_reg13_sniff_thresX,set_reg13_sniff_thresY,set_reg13_sniff_thresZ);
}
