/* USER CODE BEGIN Header */
#define TEC_ON	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define LD_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define TEC_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define LD_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define DMA_BUF_SIZE	512

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads1120.h"
#include "DAC8563.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_print(u8 data);
void LD_Init();

//协议命令定义,保持等长指令,�?后一位是校验�?
u8 cmd_len = 8;
u8 CMD_LD_OPEN[] 		 			= {0xEF, 0xEF, 0x03, 0xFF, 0x04, 0x00, 0x00, 0xE4};
u8 CMD_LD_OFF[]  		 			= {0xEF, 0xEF, 0x03, 0xFF, 0x05, 0x00, 0x00, 0xE5};
u8 CMD_LD_SET_CURR[] 			= {0xEF, 0xEF, 0x05, 0xFF, 0x00, 0x00, 0x00, 0xE4}; //第五位为设置值高八位，第六位为低八位
u8 CMD_LD_GET_CURR[] 			= {0xEF, 0xEF, 0x03, 0xFF, 0x01, 0x00, 0x00, 0xE1};
u8 CMD_LD_SET_TEMP[] 			= {0xEF, 0xEF, 0x04, 0xFF, 0x00, 0x00, 0x00, 0xE4};
u8 CMD_LD_GET_TEMP[] 			= {0xEF, 0xEF, 0x03, 0xFF, 0x02, 0x00, 0x00, 0xE2};
u8 CMD_LD_SET_CURR_MAX[] 	= {0xEF, 0xEF, 0x06, 0xFF, 0x00, 0x00, 0x00, 0xE4};	//设置�?大设置电�?
u8 CMD_LD_GET_PD1[]				= {0xEF, 0xEF, 0x03, 0xFF, 0x03, 0x00, 0x00, 0xE3};
u8 CMD_LD_GET_PD2[]				= {0xEF, 0xEF, 0x03, 0xFF, 0x06, 0x00, 0x00, 0xE6};

//返回值保持等�?
u8 return_len = 5;
u8 LD_OPEN_RETURN[] 					= {0xEF, 0x04, 0x01, 0x00, 0xEF};
u8 LD_OFF_RETURN[]  					= {0xEF, 0x05, 0x00, 0x00, 0xEF};
u8 LD_SET_CURR_RETURN[] 			= {0xEF, 0x00, 0x05, 0x00, 0xEF};	//第三位为设置状�??
u8 LD_SET_CURR_OF_RETURN[] 		= {0xEF, 0x00, 0x05, 0x01, 0xEF};	//超出�?大设置范�?
u8 LD_SET_CURR_NOTON_RETURN[] = {0xEF, 0x00, 0x05, 0xFF, 0xEF};	//LD没开
u8 LD_SET_TEMP_RETURN[] 			= {0xEF, 0x00, 0x04, 0x00, 0xEF};
u8 LD_GET_CURR_RETURN[] 			= {0xEF, 0x01, 0x00, 0x00, 0xEF};
u8 LD_GET_TEMP_RETURN[] 			= {0xEF, 0x02, 0x00, 0x00, 0xEF};
u8 LD_GET_PD1_RETURN[] 				= {0xEF, 0x03, 0x00, 0x00, 0xEF};
u8 LD_GET_PD2_RETURN[] 				= {0xEF, 0x06, 0x00, 0x00, 0xEF};
u8 LD_SET_MAXCUR_RETURN[]			=	{0xEF, 0x00, 0x06, 0x00, 0xEF};

u8 LD_SET_OUTMODE_RETURN[]		=	{0xEF, 0x07, 0x00, 0x00, 0xEF};
u8 LD_SET_PDSRC_RETURN[]			=	{0xEF, 0x08, 0x00, 0x00, 0xEF};
u8 LD_PD_ERR_RETURN[]					=	{0xEF, 0xFF, 0x00, 0x00, 0xEF};

u8 VALUE_RETURN[5];


// 变量定义
u8 	uart_revbuff[DMA_BUF_SIZE];
u32 rx_len;
u8 	rev_end_flag;
u8 	frame_tail;
u8 	i;
u16 temp_get, curr_get, curr_set, pd1_get, pd2_get;
u8 	value_high_8bit, value_low_8bit;

u8 output_update_en = 0;
u8 output_mode;

u8 pid_update;

u8 IS_ON = 0;
u16 max_cur;	//�?大设置电流，默认为最大，1.5A

//用于进入分支语句，处理不同的指令请求
const u8 get_cur 		= 0x01;
const u8 get_temp 	= 0x02;
const u8 ld_on 			= 0x04;
const u8 ld_off 		= 0x05;
const u8 set_value	= 0x00;
const u8 set_cur 		= 0x05;
const u8 set_maxcur = 0x06;
const u8 set_temp 	= 0x04;
const u8 get_pd1		=	0x03;
const u8 get_pd2		=	0x06;

const u8 out_mode_sel  	= 0x07;
const u8 pd_select  		= 0x08;


u8 send_test[1];

// ********  PID恒定功率输出   **********************
//结构体定�?
struct _pid{
	u16 SetValue;				//定义设定�?
	float ActualValue;		//定义实际�?
	float err;						//定义偏差�?
	float err_next;				//定义上一个偏差�??
	float err_last;				//定义�?上前的偏差�??
	u16 max_increment;	//�?大增�?
	u16 max_current; //目标�?大�??
	float Kp, Ki, Kd;			//定义比例、积分�?�微分系�?
	u16 err_div; //反馈量除系数
}pid;

// 实验参数，比�? = PD回读�?/设置电流�?
float	Ratio_pd_cur = 7.714;

// pid功能�?�?
u8 	pid_on = 0;

// 稳定值设�?
u16 pid_set_value = 1;
u16 pd_sample_init;

// 稳定模式选择
u8	mode_sel = 0;
const	u8	MODE_CONST_CUR 		= 0;
const	u8	MODE_CONST_PWR 		= 1;
const	u8	MODE_SAMPLE_HOLD 	= 2;

//	反馈源�?�择
const u8	PD_SEL_INT	= 0;
const u8	PD_SEL_EXT	= 1;
u8	PD_sel = PD_SEL_INT;

// 反馈源异常检�?
u16 pd_value_last, pd_value_pres;
u16 delta_pd_value;
u16 delta_threshold = 1000;
u8 pd_int_err, pd_ext_err; //pd异常标志�?
u8 curr_is_changed = 0;
u16 increment = 0;
u8 pid_zero_counter = 0;

// pid函数
void PD_SRC_ERR_DETECT(void);
void PID_init(void);
void PID_set_value(u16 set_value, u16 curr_set);
u16 PID_PD_feedback(u16 PD_value, u16 cur_set);
u16 PID_const_pw_output(u16 cur_set);

// ************************************************

// debug模式
u8 debug_mode = 0;
u16 dbg_curr;
u16 dbg_pd_int;
u16 dbg_pd_ext;
u8 dbg_curr_low;
u8 dbg_curr_high;
u8 dbg_pd_ext_low;
u8 dbg_pd_ext_high;
u8 dbg_pd_int_low;
u8 dbg_pd_int_high;
//debug模式返回�?
u8 DBG_CURR_RETURN[]	=	{0xAA, 0x55, 0x01, 0x00, 0x00};
u8 DBG_PD_INT_RETURN[]	=	{0xAA, 0x55, 0x02, 0x00, 0x00};
u8 DBG_PD_EXT_RETURN[]	=	{0xAA, 0x55, 0x03, 0x00, 0x00};
void DBG_RETURN(u8 sel);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	u8 test;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

	ADS1120_Init();
	HAL_Delay(1);
	DAC_INIT();
	HAL_Delay(1);
	u16 Vout;
	PID_init();
	LD_Init();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//if(debug_mode) DBG_RETURN(); // debug模式返回
		
		//curr_get = GET_LD_CURR();
    if(rev_end_flag)
		{
			// ========== 进入debug模式使用，调节pid参数 =========================
			if(uart_revbuff[0] == 0x55 && uart_revbuff[1] == 0xaa)
			{
				if(uart_revbuff[2] == 1)debug_mode = 1;
				else if(uart_revbuff[2] == 0)debug_mode = 0;
				switch(uart_revbuff[2])
					{
						case 0x02:
							pid.Kp = (float)(uart_revbuff[3]*256 + uart_revbuff[4])/1000;
							break;
						case 0x03:
							pid.Ki = (float)(uart_revbuff[3]*256 + uart_revbuff[4])/1000;
							break;
						case 0x04:
							pid.Kd = (float)(uart_revbuff[3]*256 + uart_revbuff[4])/1000;
							break;
						case 0x05:
							pid.err_div = uart_revbuff[3];
							break;
						default: break;
					}
			}
			//===================================================================
			//提示信号收发
			
			if(uart_revbuff[0] == 0xef && uart_revbuff[1] == 0xef)	//帧头校验
			{
				for(i = 0; i<rx_len-1; i++)										//获取帧尾
				{
					frame_tail = frame_tail + uart_revbuff[i];
				}
				if(frame_tail == uart_revbuff[rx_len-1])			//帧尾校验
				{
					//rev_end_flag = 0;	//标志位置�?
					switch(uart_revbuff[4])
					{
						case get_cur:
							curr_get = GET_LD_CURR();
							value_low_8bit  = curr_get & 0xff;
							value_high_8bit = (curr_get>>8) & 0xff;

							LD_GET_CURR_RETURN[2] = value_high_8bit;
							LD_GET_CURR_RETURN[3] = value_low_8bit;
							HAL_UART_Transmit_DMA(&huart1, LD_GET_CURR_RETURN, return_len);
							break;
						
						case get_temp:
							temp_get = GET_LD_TEMP();
							value_low_8bit  = temp_get & 0xff;
							value_high_8bit = (temp_get>>8) & 0xff;
							LD_GET_TEMP_RETURN[2] = value_high_8bit;
							LD_GET_TEMP_RETURN[3] = value_low_8bit;	
						
							HAL_UART_Transmit_DMA(&huart1, LD_GET_TEMP_RETURN, return_len);
							break;
						
						case get_pd1:
							pd1_get = GET_LD_PD_internal();
							value_low_8bit  = pd1_get & 0xff;
							value_high_8bit = (pd1_get>>8) & 0xff;
							LD_GET_PD1_RETURN[2] = value_high_8bit;
							LD_GET_PD1_RETURN[3] = value_low_8bit;	
							HAL_UART_Transmit_DMA(&huart1, LD_GET_PD1_RETURN, return_len);
							break;
						
						case get_pd2:
							pd2_get = GET_LD_PD_external();
							value_low_8bit  = pd2_get & 0xff;
							value_high_8bit = (pd2_get>>8) & 0xff;
							LD_GET_PD2_RETURN[2] = value_high_8bit;
							LD_GET_PD2_RETURN[3] = value_low_8bit;	
						
							HAL_UART_Transmit_DMA(&huart1, LD_GET_PD2_RETURN, return_len);
							break;
						
						case ld_on:
							LD_ON;
							TEC_ON;
							HAL_GPIO_WritePin(run_state_GPIO_Port, run_state_Pin, GPIO_PIN_SET);
							IS_ON = 1;
							Vout = 0x0000;
							DAC_Set_Voltage(Vout, ADDR_DAC_A);
							HAL_UART_Transmit_DMA(&huart1, LD_OPEN_RETURN, return_len);
							break;
						
						case ld_off:
							DAC_Set_Current(0x0000, ADDR_DAC_A);
							LD_OFF;
							IS_ON = 0;
							HAL_GPIO_WritePin(run_state_GPIO_Port, run_state_Pin, GPIO_PIN_RESET);
							
							HAL_UART_Transmit_DMA(&huart1, LD_OFF_RETURN, return_len);
							break;
						
						case pd_select:
							PD_sel = uart_revbuff[6];
							LD_SET_PDSRC_RETURN[3] = PD_sel;
							LD_PD_ERR_RETURN[3] = PD_sel;
							// �?测所选pd是否异常
							if(pd_ext_err == 1 && PD_sel == PD_SEL_EXT)
							{
								HAL_UART_Transmit_DMA(&huart1, LD_PD_ERR_RETURN, return_len);
							}
							else if(pd_int_err == 1 && PD_sel == PD_SEL_INT)
							{
								HAL_UART_Transmit_DMA(&huart1, LD_PD_ERR_RETURN, return_len);
							}
							else HAL_UART_Transmit_DMA(&huart1, LD_SET_PDSRC_RETURN, return_len);
							break;
						
						case out_mode_sel:
							mode_sel = uart_revbuff[6];
							if(mode_sel != MODE_CONST_CUR)
							{
								HAL_TIM_Base_Start_IT(&htim16);
								pid_on = 1;
							}
							else
							{								
								pid_on = 0;
								HAL_TIM_Base_Stop_IT(&htim16);
							}
							LD_SET_OUTMODE_RETURN[3] = mode_sel;
							HAL_UART_Transmit_DMA(&huart1, LD_SET_OUTMODE_RETURN, return_len);
							
							break;
						
						case set_value:
							switch(uart_revbuff[2])
							{
								case set_temp://设置温度
									break;
								
								case set_cur://设置电流
									if(IS_ON)
									{
										curr_set = uart_revbuff[5];									//设置值高字节
										curr_set = (curr_set<<8) | uart_revbuff[6];	//设置值低字节
										if(curr_set > max_cur) //	�?大设置�?�判�?
										{
											HAL_UART_Transmit_DMA(&huart1, LD_SET_CURR_OF_RETURN, return_len);
											break;
										}
										
										output_update_en = 1;			//重新设定pid目标�?
										curr_is_changed = 1;			// flag for pd error detection
										DAC_Set_Current(curr_set>>1, ADDR_DAC_A);
										HAL_UART_Transmit_DMA(&huart1, LD_SET_CURR_RETURN, return_len);
										break;
									}
									else
									{
										HAL_UART_Transmit_DMA(&huart1, LD_SET_CURR_NOTON_RETURN, return_len);
										break;
									}

								case set_maxcur:
									max_cur = uart_revbuff[5];									//设置值高字节
									max_cur = (max_cur<<8) | uart_revbuff[6];	  //设置值低字节
									HAL_UART_Transmit_DMA(&huart1, LD_SET_MAXCUR_RETURN, return_len);
									break;
								default:break;
							
							}
							break;
						default:
							break;
					}
				}
				else
				{
					frame_tail = 0;
					rev_end_flag = 0;
				}
			}
			else
			{
				frame_tail = 0;
				rev_end_flag = 0;
			}
			//重新�?始dma传输
			HAL_UART_Receive_DMA(&huart1, uart_revbuff, DMA_BUF_SIZE);	
		}
		
		// ============ 输出模式选择：恒电流/恒功�?/采样保持 ========================
		if(pid_on)
		{
				// 棿测异常pd
			PD_SRC_ERR_DETECT();
			pd_value_pres = (PD_sel == PD_SEL_INT) ? GET_LD_PD_internal() : GET_LD_PD_external();
			// 更新设置�?
			if(output_update_en)
			{
				//DAC_Set_Voltage(curr_set, ADDR_DAC_A);
				pd_value_pres = (PD_sel == PD_SEL_INT) ? GET_LD_PD_internal() : GET_LD_PD_external();
				switch(mode_sel)
				{
					case MODE_CONST_PWR: 
						pid_set_value = (u16)(curr_set * Ratio_pd_cur);
						PID_set_value(pid_set_value, curr_set);
						output_update_en = 0;
						break;
					case MODE_SAMPLE_HOLD: 
						pid_set_value = pd_value_pres;
						PID_set_value(pid_set_value, curr_set);
						output_update_en = 0;
						break;
					default:
						output_update_en = 0;
						break;
				}
			}
			else if(pid_update)
			{
				pid_update = 0;
				pd_value_last = pd_value_pres;
				pd_value_pres = (PD_sel == PD_SEL_INT) ? GET_LD_PD_internal() : GET_LD_PD_external();
				
//				// zero detection
//				if(pd_value_pres<0x1F)
//				{
//					pid_zero_counter++;
//				}
//				if(pid_zero_counter>10)
//				{
//					// no pd feedback
//					HAL_TIM_Base_Stop_IT(&htim16);
//					pid_on = 0;
//					pid_update = 0;
//					pid_zero_counter = 0;
//				}
				
				
				// 恒功率输�?,每次循环均调整电流输出�??;集成pd异常�?�?
				increment = PID_PD_feedback(pd_value_pres, curr_set);
				curr_set = ((increment&0xC000) != 0) ? (curr_set - 0xffff + increment) : curr_set + increment;
				
				if(curr_set > pid.max_current)
				{
					curr_set = pid.max_current;
				}
				DAC_Set_Voltage(curr_set>>1, ADDR_DAC_A);
			}
			
			
		}
		// ======================================================================================
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LD_Init(){
	
	u16 Init_Cur, Init_Temp;
	Init_Cur = 0x0000;
	Init_Temp = 0x7fff;
	LD_OFF;
	TEC_ON;
	DAC_Set_Voltage(Init_Cur, ADDR_DAC_A);
	DAC_Set_Voltage(Init_Temp, ADDR_DAC_B);
	max_cur = 0xFFFF; //限制�?大电�?
}


void uart_print(u8 data){
	u8* send;
	*send = data;
	HAL_UART_Transmit(&huart1, send, 1, 1000);
}

//**********  PID恒定功率输出函数实现 *************************************
// PID 初始�?
void PID_init(){
	pid.SetValue		= 0;
	pid.ActualValue	= 0;
	pid.err					= 0;
	pid.err_last		=	0;
	pid.err_next		=	0;
	pid.Kp					=	0.2;
	pid.Ki					=	0.00000001;
	pid.Kd					=	0.00000001;
	pid.max_increment		= 40;
	pid.max_current	=	0;
	pid.err_div     = 4;
};

// 确定设置�?
void PID_set_value(u16 set_value, u16 curr_set)
{
	pid.SetValue 				= set_value;
	pid.max_current			=	curr_set + 0x1999;
}

// 获取反馈量并调整电流设置�?

u16 PID_PD_feedback(u16 PD_value, u16 cur_set){
	//u16 cur_update = 0x00;
	float incrementValue, dest_value;
	//unsigned int test;
	pid.ActualValue	=	PD_value;
	pid.err 				= (pid.SetValue - pid.ActualValue)/pid.err_div;
  incrementValue 	= pid.Kp*pid.err + pid.Ki*(pid.err+pid.err_next+pid.err_last) + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);//计算出增�?
	dest_value			=	cur_set + incrementValue;

	if(incrementValue > pid.max_increment)
	{
		incrementValue = pid.max_increment;
	}
	else if(-incrementValue > (pid.max_increment))
	{
		incrementValue = 0 - pid.max_increment;
	}
	else
	{
		incrementValue = incrementValue;
	}
	
	pid.err_last 		= pid.err_next;
	pid.err_next 		= pid.err;
	return incrementValue;
}

// PID恒定功率输出实现
u16 PID_const_pw_output(u16 cur_set)
{
	u16 PD_value;
	u16 cur_update;
	PD_value 				= (PD_sel == PD_SEL_INT) ? GET_LD_PD_internal() : GET_LD_PD_external();
	cur_update 				=	PID_PD_feedback(PD_value, cur_set);
	// 设置新电流�??
	//DAC_Set_Current(cur_update, ADDR_DAC_A);
	return cur_update;
}

// PD反馈源异常检�? + 自动切换反馈�?
void PD_SRC_ERR_DETECT(void)
{
		pd_value_pres = (PD_sel == PD_SEL_INT) ? GET_LD_PD_internal() : GET_LD_PD_external();
		
		if(curr_is_changed)
		{
			curr_is_changed = 0;
			pd_value_last = pd_value_pres;
			return;
		}	
		else
		{
			delta_pd_value = (pd_value_last > pd_value_pres) ? (pd_value_last-pd_value_pres) : (pd_value_pres-pd_value_last) ;
		}
	
		if(delta_pd_value > delta_threshold)	//�?测到异常
		{
			if(PD_sel == PD_SEL_EXT) pd_ext_err = 1;
			else pd_int_err = 1;
			
			LD_PD_ERR_RETURN[3] = PD_sel;
			HAL_UART_Transmit_DMA(&huart1, LD_PD_ERR_RETURN, return_len);

			pid_on = 0;																// 若反馈源寄则关闭pid
			mode_sel = MODE_CONST_CUR;								// 设置为恒电流模式
			DAC_Set_Current(curr_set, ADDR_DAC_A); 		// 将当前电流�?�作为输出�??
		}
		
		pd_value_last = pd_value_pres;
}
//***************************************************************************

// *************************** debug模式，返回下位机数据 ******************************
void DBG_RETURN(u8 sel)
{
	dbg_curr = GET_LD_CURR();
	dbg_pd_ext = GET_LD_PD_external();
	dbg_pd_int = GET_LD_PD_internal();
	
	dbg_curr_low = dbg_curr & 0xff;
	dbg_curr_high = (dbg_curr>>8) & 0xff;
	DBG_CURR_RETURN[3] = dbg_curr_high;
	DBG_CURR_RETURN[4] = dbg_curr_low;	
	
	dbg_pd_int_low = dbg_pd_int & 0xff;
	dbg_pd_int_high = (dbg_pd_int>>8) & 0xff;
	DBG_PD_INT_RETURN[3] = dbg_pd_int_high;
	DBG_PD_INT_RETURN[4] = dbg_pd_int_low;	
	
	dbg_pd_ext_low = dbg_pd_ext & 0xff;
	dbg_pd_ext_high = (dbg_pd_ext>>8) & 0xff;
	DBG_PD_EXT_RETURN[3] = dbg_pd_ext_high;
	DBG_PD_EXT_RETURN[4] = dbg_pd_ext_low;	
	
	switch(sel)
	{
		case 0x07:
			HAL_UART_Transmit_DMA(&huart1, DBG_CURR_RETURN, 5);
			break;
		case 0x08:
			HAL_UART_Transmit_DMA(&huart1, DBG_PD_INT_RETURN, 5);
			break;
		case 0x09:
			HAL_UART_Transmit_DMA(&huart1, DBG_PD_EXT_RETURN, 5);
			break;
		default:break;
	}
	
	
	
}

// 可进行代码复�?
//void DBG_LOAD_DATA(u16 value, u8*data_return)
//{
//	u8 low, high;
//	dbg_curr_low = dbg_curr & 0xff;
//	dbg_curr_high = (dbg_curr>>8) & 0xff;
//	DBG_CURR_RETURN[3] = high;
//	DBG_CURR_RETURN[4] = low;	
//}
// ***********************************************************************************
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
