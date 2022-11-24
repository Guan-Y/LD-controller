/*
USE IC: 				ADS1120	 16bit-HighPrecision-ADC
Communication:	SPI1
*/
#include "ads1120.h"
#include "spi.h"
//#include "gpio.h"
#include "main.h"
#include <stdio.h>
#define DRDYn  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define CSn HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET)
#define CS  HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);

/*
ADS1120 -- 单片机接口
*******************************
*		ADS1120 			单片机			*
*		CS			<=>		PA3					*
*		DRDY#		<=>		PB0					*
*		SCLK		<=>		PA5 (SCK)		*
*		DOUT		<=>		PA6	(MISO)	*
*		DIN			<=>		PA7	(MOSI)	*
*******************************
*/
#define BUFFSIZE 3
u8 psend[2];
u8 prev[2];
u8 CMD_Send[1];

u8 revbuff[BUFFSIZE];

// debug模式
extern u8 debug_mode;
//debug模式返回值
extern u8 DBG_CURR_RETURN[];
extern u8 DBG_PD_INT_RETURN[];
extern u8 DBG_PD_EXT_RETURN[];



//初始化设置
void ADS1120_Init(void)
{
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);
	ADS1120_Send_CMD(ADS1120_CMD_RESET);		//复位
	HAL_Delay(10);
//	ADS1120_Set_Channel(CHANNEL_AIN0|GAIN_1);			//默认使用0通道，增益1
//	ADS1120_Set_REF(VREF_REF0);							//默认参考电压取自REF0
//	ADS1120_DRDY_AF_CMD(DISABLE);						//默认不使用复用方式
//	ADS1120_Config_Mode(MODE_Normal|CM_Continuous);				//默认normal模式，速率20SPS
//	ADS1120_Write_REG(REG3_ADDR,0x24);
	
	ADS1120_Init_WRREG(REG0_ADDR, 0x80);
	ADS1120_Init_WRREG(REG1_ADDR, 0xD0); //110_10_100
	ADS1120_Init_WRREG(REG2_ADDR, 0x40);
	ADS1120_Init_WRREG(REG3_ADDR, 0x00);
	
	ADS1120_Send_CMD(ADS1120_CMD_START);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);

}

void ADS1120_Init_WRREG(u8 REG_ADDR, u8 Data)
{
	u8 wcmd;
	u8 p_rev[2];
	wcmd = ADS1120_CMD_WREG | (REG_ADDR);
	psend[0] = wcmd;
	psend[1] = Data;
	
	HAL_SPI_TransmitReceive(&hspi1, psend,p_rev, 2, 5000);
}

//发送指令给ADS1120
void ADS1120_Send_CMD(u8 CMD)
{
	CMD_Send[0] = CMD;
//	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,CMD_Send,1,1000);
//	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);
}

//写ADS1120的寄存器
void ADS1120_Write_REG(u8 REG_ADDR, u8 Data)
{
	//u8 psend[2];
	u8 wcmd;
	u8 p_rev[2];
	wcmd = ADS1120_CMD_WREG | (REG_ADDR);
	psend[0] = wcmd;
	psend[1] = Data;
	
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);	
	HAL_SPI_TransmitReceive(&hspi1, psend,p_rev, 2, 5000);
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);	
}

//读ADS1120的寄存器
u8 ADS1120_READ_REG(u8 REG_ADDR)
{
	u8 revreg;
	u8 psend[2];
	psend[0] = ADS1120_CMD_RREG | (REG_ADDR); //发送命令
	psend[1] = 0x00;													//空写

	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);	//拉低片选cs  
	HAL_SPI_TransmitReceive(&hspi1, psend, prev, 2, 1000);
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);			//拉高片选，锁存
	revreg = prev[1];		//取数据
	return revreg;
}

//设置工作模式
void ADS1120_Config_Mode(u8 mode)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG1_ADDR);
	ADS1120_Write_REG(REG1_ADDR, rev|mode);
}

//设置数据传输速率
void ADS1120_Config_DataRate(u8 datarate)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG1_ADDR);
	ADS1120_Write_REG(REG1_ADDR, rev|datarate);
}

//设置采样通道
void ADS1120_Set_Channel(u8	Channel)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG0_ADDR);
	rev = rev&0x0f;
	ADS1120_Write_REG(REG0_ADDR, rev|Channel);
	//HAL_Delay(5);
}

//设置通道增益
void ADS1120_Set_Gain(u8	gain)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG0_ADDR);
	ADS1120_Write_REG(REG0_ADDR, rev|gain);
}

//开启or关闭PGA
void ADS1120_PGA_Cmd(u8 status)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG0_ADDR);
	ADS1120_Write_REG(REG0_ADDR, rev|status);
}
	
//设置参考电压源
void ADS1120_Set_REF(u8 REFSRC)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG2_ADDR);
	ADS1120_Write_REG(REG2_ADDR, rev|REFSRC);
}	

//使能or除能DRDY数据复用
void ADS1120_DRDY_AF_CMD(u8 STATUS)
{
	u8 rev;
	rev = ADS1120_READ_REG(REG3_ADDR);
	ADS1120_Write_REG(REG3_ADDR, rev|(STATUS<<1));
}

//读取转换结果数据
u16 ADS1120_ReadData(void)
{
	u16 Revdata;
	u8 psend[2];
	//装入发送的命令，空写两个字节
	psend[0] = 0xff;
	psend[1] = 0xff;
	
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);	
	while(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin));
	HAL_SPI_TransmitReceive(&hspi1, psend, revbuff, 2, 50);
	Revdata = revbuff[0]&0x00ff;
	Revdata = (Revdata<<8) | revbuff[1];
	
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);	
	return Revdata;
}

//single shot模式读取数据
//读取转换结果数据
u16 ADS1120_ReadData_singleshot(void)
{
	u16 Revdata;
	u8 psend[2];
	//装入发送的命令，空写两个字节
	psend[0] = 0xff;
	psend[1] = 0xff;

	
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);	
	ADS1120_Send_CMD(ADS1120_CMD_START); // START/SYNC
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);
	
	while(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin)); //Data Ready
	
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, psend, revbuff, 2, 5000); // Readback
	Revdata = revbuff[0]&0x00ff;
	Revdata = (Revdata<<8) | revbuff[1];
	HAL_GPIO_WritePin(ADC_NCS_GPIO_Port, ADC_NCS_Pin, GPIO_PIN_SET);	
	return Revdata;
}

//读取LD驱动电流
u16 GET_LD_CURR(void){
	u16 value;
	ADS1120_Set_Channel(CHANNEL_AIN0|GAIN_1);
//	ADS1120_Send_CMD(ADS1120_CMD_START);

	value = ADS1120_ReadData_singleshot();
	return value;
} 

//读取LD温度
u16 GET_LD_TEMP(void){
	u16 value;
	ADS1120_Set_Channel(CHANNEL_AIN1|GAIN_1);
//	ADS1120_Send_CMD(ADS1120_CMD_START);

	value = ADS1120_ReadData_singleshot();
	return value;
} 

//读取PD1电流
u16 GET_LD_PD_internal(void){
	u16 value;
	ADS1120_Set_Channel(CHANNEL_AIN2|GAIN_1);
//	ADS1120_Send_CMD(ADS1120_CMD_START);

	value = ADS1120_ReadData_singleshot();
	return value;
} 

//读取PD2电流
u16 GET_LD_PD_external(void){
	u16 value;
	ADS1120_Set_Channel(CHANNEL_AIN3|GAIN_1);
//	ADS1120_Send_CMD(ADS1120_CMD_START);
	value = ADS1120_ReadData_singleshot();
	return value;
} 