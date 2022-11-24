#include "main.h"
//24位数据：任意码（2位） 命令码（3位） 地址码（3位） 输入数据（16位）

#define CMD_WR_REG 				0x000000
#define CMD_UD_DAC				0x080000
#define CMD_WR_REG_UD_ALL	0x100000
#define CMD_WR_REG_UD			0x180000
#define CMD_PWR_SET				0x200000
#define CMD_SOFT_RST			0x280000
#define CMD_SET_LDAC			0x300000
#define CMD_INTERNAL_REF	0x380000


#define ADDR_DAC_A		0x000000
#define ADDR_DAC_B		0x010000
#define ADDR_GAIN			0x020000
#define ADDR_DAC_AB		0x070000



#define PWR_UP 				0x000000
#define PWR_DOWN_1k		0x000008
#define PWR_DOWN_100k	0x000010
#define PWR_DOWN_HIZ	0x000018

#define DAC_A			0x000001
#define DAC_B 		0x000002
#define DAC_AB		0x000003

#define GAIN_B2A2	0x000000
#define GAIN_B2A1	0x000001
#define GAIN_B1A2	0x000002
#define GAIN_B1A1	0x000003

//函数定义
void DAC8563_GAIN_SET(u32 gain_set);
void DAC8563_PWR_SET(u32 mode, u32 dac_x);
void DAC8563_REF_SET(u32 status);
void DAC8563_UPDATE(u32 addr);
void DAC8563_WR_REG(u32 addr, u16 data);
void DAC8563_WR_REG_UD_ALL(u32 addr, u16 data);
void DAC8563_WR_REG_UPDATE(u32 addr, u16 data);

void DAC_Set_Current(u16 cur_set,u32 channel);
void DAC_Set_Voltage(u16 V_set, u32 channel);
void DAC_INIT(void);