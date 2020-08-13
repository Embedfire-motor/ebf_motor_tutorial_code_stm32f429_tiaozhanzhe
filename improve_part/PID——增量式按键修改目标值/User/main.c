/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   GPIO输出--使用固件库点亮LED灯
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32 F429 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "./tim/bsp_basic_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./key/bsp_key.h" 

extern _pid pid;
extern float set_point;
extern int pid_status;


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
	/* 初始化系统时钟为180MHz */
	SystemClock_Config();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	/* 协议初始化 */
	protocol_init();
	DEBUG_USART_Config();
	/* 初始化基本定时器定时，20ms产生一次中断 */
	TIMx_Configuration();
	/* PID算法参数初始化 */
	PID_param_init();	
	/*led初始化*/
	LED_GPIO_Config();
	/*按键初始化*/
	Key_GPIO_Config();
	int run_i=0;
	
#if defined(PID_ASSISTANT_EN) 
    int temp = set_point;    // 上位机需要整数参数，转换一下
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
#endif
	while(1)
	{
		/* 接收数据处理 */
		receiving_process();
		/*模拟修改PID目标值*/
		if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
				if(run_i%2==0)
				{
						set_point=200;
						pid.target_val = set_point;
				}
				else
				{
						set_point=0;
						pid.target_val = set_point;
				}
				run_i++;
#if defined(PID_ASSISTANT_EN) 
				temp = set_point;    // 上位机需要整数参数，转换一下
				set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
#endif
		}   
		
		if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
			pid_status=!pid_status;//取反状态
		  
#if defined(PID_ASSISTANT_EN) 
		  if (!pid_status)
		  {
			set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
		  }
		  else
		  {
			set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
		  }      
#endif
		} 
	}
}



/**
  * @brief  系统时钟配置 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  无
  * @retval 无
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* 使能HSE，配置HSE为PLL的时钟源，配置PLL的各种分频因子M N P Q 
	 * PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
	 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   while(1) {}
  }

  /* 激活 OverDrive 模式 */
  HAL_PWREx_EnableOverDrive();
 
  /* 选择PLLCLK作为SYSCLK，并配置 HCLK, PCLK1 and PCLK2 的时钟分频因子 
	 * SYSCLK = PLLCLK     = 180M
	 * HCLK   = SYSCLK / 1 = 180M
	 * PCLK2  = SYSCLK / 2 = 90M
	 * PCLK1  = SYSCLK / 4 = 45M
	 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {}
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
