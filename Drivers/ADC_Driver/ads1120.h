#include "main.h"
//ADS1120 Resource 
//复用脚使能
#define ENABLE	0x01
#define DISABLE	0x00

//Register MAP
#define REG0_ADDR	0x00
#define REG1_ADDR	0x04
#define REG2_ADDR	0x08
#define REG3_ADDR	0x0C

//COMMAND
#define ADS1120_CMD_RESET	0x06
#define ADS1120_CMD_START	0x08
#define ADS1120_CMD_SYNC	0x08
#define ADS1120_CMD_PWRDOWN	0x02
#define ADS1120_CMD_RDATA	0x10
#define ADS1120_CMD_RREG	0x20
#define ADS1120_CMD_WREG	0x40

//CHANNEL PGA_GAIN SET (Register0)
#define CHANNEL_AIN0	0x80
#define CHANNEL_AIN1	0x90
#define CHANNEL_AIN2	0xA0
#define CHANNEL_AIN3	0xB0

#define GAIN_1		0x00
#define GAIN_2		0x02
#define GAIN_4		0x04
#define GAIN_8		0x06
#define GAIN_16		0x08
#define GAIN_32		0x0A
#define GAIN_64		0x0C
#define GAIN_128	0x0E

#define PGA_OFF		0x01


//DR MODE ConversionMode TS BCS(Register1)
#define DataRate0	0x00
#define DataRate1	0x20
#define DataRate2	0x40
#define DataRate3	0x60
#define DataRate4	0x80
#define DataRate5	0xA0
#define DataRate6	0xC0
#define DataRate7	0xE0

#define MODE_Normal			0x00
#define MODE_Duty_Cycle	0x08
#define MODE_Turbo			0x10
#define MODE_Reserve		0x18

#define CM_Single_Shot	0x00
#define CM_Continuous		0x04

#define TS_DISABLE	0x00
#define TS_ENABLE		0x02

#define BCS_OFF			0x00
#define BCS_ON			0x01

//VREF FIR PWR IDAC Config(Register2)
#define VREF_INT					0x00
#define VREF_REF0					0x40
#define VREF_REF1					0x80
#define VREF_ANALOG_SUP		0xC0

#define PSW_SW_OPEN				0x00
#define PSW_SW_AUTO				0x08



//DRDYM	(Register3)
#define DRDY_AF			0x02
#define DRDY_ONLY		0x00

void ADS1120_Init(void);
void ADS1120_Send_CMD(u8 CMD);
void ADS1120_Write_REG(u8 REG, u8 Data);
void ADS1120_Config_Mode(u8 mode);
void ADS1120_Set_Channel(u8	Channel);
void ADS1120_Set_REF(u8 REFSRC);
void ADS1120_DRDY_AF_CMD(u8 STATUS);
void ADS1120_Set_Gain(u8	gain);
void ADS1120_Config_DataRate(u8 datarate);
void ADS1120_PGA_Cmd(u8 status);

void ADS1120_Init_WRREG(u8 REG_ADDR, u8 Data);


u8 ADS1120_READ_REG(u8 REG_ADDR);
u16 ADS1120_ReadData();


//读取LD驱动电流
u16 GET_LD_CURR(void);

//读取LD温度
u16 GET_LD_TEMP(void);

//读取PD1电流
u16 GET_LD_PD_internal(void);

//读取PD2电流
u16 GET_LD_PD_external(void);