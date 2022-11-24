#include "DAC80501.h"
#include "DAC8563.h"
#include "spi.h"
#define DAC8563	EN
#define SPI	0
#define I2C	1



/*
DAC8830 -- 单片机接口
*****************************************
*		DAC8830 			单片机								*
*		SCLK		<=>		PA5 (SCK)	/ PB6(SCL)	*
*		CS			<=>		PB2										*
*		SDI			<=>		PA7	(MOSI) / PB7(SDA)	*
*****************************************
*/

/*
DAC8563 -- 单片机接口
*****************************************
*		DAC8563 			单片机								*
*		SCLK		<=>		PA5 (SCK)	/ PB6(SCL)	*
*		NSYNC		<=>		PB11									*
*		SDI			<=>		PA7	(MOSI) / PB7(SDA)	*
*		NCLR		<=>		PB10									*	
*		NLDAC		<=>		PB2										*
*****************************************

*/

u8 datasend[3];


//DAC外设初始化
void DAC_INIT(void)
{
	#ifndef DAC8563
	
	#else
	DAC_SOFT_RESET();
	HAL_Delay(10);
	DAC8563_PWR_SET(PWR_UP, DAC_AB);
	DAC8563_REF_SET(DISABLE);		//DISABLE用外部基准
	DAC8563_GAIN_SET(GAIN_B1A1);
	DAC8563_WR_REG_UD_ALL(ADDR_DAC_AB,0x0000);
	DAC_Set_Voltage(0x00, ADDR_DAC_AB);
	#endif
}

	

//设置同步/异步模式
void SET_SYNC_MODE(u16 STATUS)
{
	u32 psend;
	psend = CMD_SYNC|STATUS;
	DAC_send_data(psend);
}

//DAC软复位
void DAC_SOFT_RESET(void)
{
	u32 psend;
	psend = CMD_SOFT_RST|0x000001;
	DAC_send_data(psend);
}

//低功耗设置
void DAC_PWR_CONFIG(u16 REFDAC)
{
	u32 psend;
	psend = CMD_CONFIG|REFDAC;
	DAC_send_data(psend);
}

//参考电压分压、输出增益设置
void SET_GAIN_DIV(u16 DIVx, u16 BUF_GAIN)
{
	u32 psend;
	psend = CMD_GAIN|DIVx|BUF_GAIN;
	DAC_send_data(psend);
}

//发送数据：单片机 => DAC
void DAC_send_data(u32 data)
{
	#ifndef DAC8563						//无定义采用dac8830
	u8 psend[2];
	psend[0] = (data>>8)&0xff; 	//取MSB
	psend[1] = data&0xff;				//取LSB
	HAL_SPI_Transmit(&hspi1,psend,2,10);
	#else											//采用dac8563
	HAL_GPIO_WritePin(DAC_NSYNC_GPIO_Port, DAC_NSYNC_Pin, GPIO_PIN_RESET);
	u8 psend[3];
	psend[0] = (data>>16)&0xff;	
	psend[1] = (data>>8)&0xff; 	
	psend[2] = data&0xff;				
	HAL_SPI_Transmit(&hspi1,psend,3,10);
	HAL_GPIO_WritePin(DAC_NSYNC_GPIO_Port, DAC_NSYNC_Pin, GPIO_PIN_SET);
	#endif

}

//用于适配实际电路，计算需发送的值
u16 get_digital(u16 cur)
{
	u16 Vout = cur;
	return Vout;
}


//设置电压 channel指所需设置的dac的地址 例如ADDR_DAC_A
void DAC_Set_Voltage(u16 V_set, u32 channel)	//输入为需要的电压值
{
	#ifndef 	DAC8563
	u16 digit_cur;
	digit_cur = get_digital(V_set);
	DAC_send_data(digit_cur);
	#else
	u32 psend;
	psend = CMD_WR_REG_UD | channel | V_set;
	DAC_send_data(psend);
	#endif
}


//设置电流 channel指所需设置的dac的地址 例如ADDR_DAC_A
void DAC_Set_Current(u16 cur_set, u32 channel)	//输入为需要的电流值
{
	#ifndef 	DAC8563
	u16 digit_cur;
	digit_cur = get_digital(cur_set);
	DAC_send_data(digit_cur);
	#else
	u16 vout;
	vout = get_digital(cur_set);
	DAC_Set_Voltage(vout, channel);
	HAL_Delay(5);
	#endif
}



//DAC8563驱动函数
void DAC8563_GAIN_SET(u32 gain_set)
{
	u32 data = CMD_WR_REG | ADDR_GAIN | gain_set;
	DAC_send_data(data);

};

void DAC8563_PWR_SET(u32 mode, u32 dac_x)
{
	u32 data = CMD_WR_REG | mode | dac_x;
	DAC_send_data(data);

};

void DAC8563_REF_SET(u32 status)
{
	u32 data = CMD_INTERNAL_REF| 0x070000 |status;
	DAC_send_data(data);

};

void DAC8563_UPDATE(u32 addr)
{
	u32 data = CMD_UD_DAC | addr;
	DAC_send_data(data);
};


void DAC8563_WR_REG(u32 addr, u16 dataw)
{
	u32 data = CMD_WR_REG| addr | dataw;
	DAC_send_data(data);
};

void DAC8563_WR_REG_UD_ALL(u32 addr, u16 dataw)
{
	u32 data = CMD_WR_REG| addr | dataw;
	DAC_send_data(data);
};

void DAC8563_WR_REG_UPDATE(u32 addr, u16 dataw)
{
	u32 data = CMD_WR_REG| addr | dataw;
	DAC_send_data(data);
};


