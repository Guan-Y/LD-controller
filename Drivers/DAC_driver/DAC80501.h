#include "main.h"

void USE_DAC80501(void);
void USE_DAC8830(void);

//寄存器地址
#define REG_NOOP_ADDR			0x0000
#define REG_DEVID_ADDR		0x0001
#define REG_SYNC_ADDR			0x0002
#define REG_CONFIG_ADDR		0x0003
#define REG_GAIN_ADDR			0x0004
#define REG_TRIGGER_ADDR	0x0005
#define REG_STATUS_ADDR		0x0007
#define REG_DAC_ADDR			0x0008


//SYNC Register 设置输出模式(同步、异步)
#define DAC_SYNC_EN		0x0001
#define DAC_ASYNC_EN	0x0000


//CONFIG Register 
#define CONFIG_REF_PWRDOWN	0x0100
#define CONFIG_DAC_PWRDOWN	0x0001

//GAIN Register
#define REF_DIV_1		0x0000
#define REF_DIV_2		0x0100
#define BUFF_GAIN_1 0x0000
#define BUFF_GAIN_2 0x0001

//TRIGGER Register
#define SYNC_LDAC		0x0008
#define SOFT_RESET	0x000A

//DAC Register
#ifdef DACx0501Z
#define DAC_RESET	0x0000
#else
#define DAC_RESET	0x8000
#endif

//命令 数据中的高四位
#define CMD_NOOP		0x00000
#define CMD_DEVID		0x10000
#define CMD_SYNC		0x20000
#define CMD_CONFIG	0x30000
#define CMD_GAIN		0x40000
#define CMD_TRIGGER	0x50000
#define CMD_STATUS	0x70000
#define CMD_SET_DAC	0x80000

void DAC_INIT(void);
void SET_DAC80501_COM(u8 com);
void SET_SYNC_MODE(u16 STATUS);
void DAC_SOFT_RESET(void);
void DAC_PWR_CONFIG(u16 REFDAC);
void SET_GAIN_DIV(u16 DIVx, u16 BUF_GAIN);
void DAC_send_data(u32 data);
u16 get_digital(u16 cur);

