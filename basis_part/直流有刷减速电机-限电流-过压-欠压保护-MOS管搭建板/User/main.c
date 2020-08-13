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
#include "./tim/bsp_motor_tim.h"
#include "./led/bsp_led.h"
#include ".\key\bsp_key.h" 
#include ".\motor_control\bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./adc/bsp_adc.h"


/* 宏定义 */
#define CURR_MAX    200    // 最大电流（单位mA）

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT*0.5;
  uint8_t curr_max_count = 0;
  uint8_t flag = 0;
  uint8_t dir = 0;

  HAL_Init();
  
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
  
	/* 初始化按键GPIO */
	Key_GPIO_Config();
  
  /* 初始化 LED */
  LED_GPIO_Config();

  /* 电机初始化 */
  motor_init();
  
  /* 串口初始化 */
  DEBUG_USART_Config();

  /* ADC 始化 */
  ADC_Init();
  
  set_motor_speed(ChannelPulse);
  set_motor_disable();    // 禁用电机
  printf("野火直流有刷电机-限流-过压-欠压保护实验\r\n");
	
	while(1)
	{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* 使能电机 */
      set_motor_enable(); 
			
			while(1)
			{
				/* 扫描KEY1 */
				if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
				{
					/* 增大占空比 */
					ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
					
					if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
						ChannelPulse = PWM_MAX_PERIOD_COUNT;
					
					set_motor_speed(ChannelPulse);
				}
				
				/* 扫描KEY2 */
				if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
				{
					if(ChannelPulse < PWM_MAX_PERIOD_COUNT/10)
						ChannelPulse = 0;
					else
						ChannelPulse -= PWM_MAX_PERIOD_COUNT/10;
					
					set_motor_speed(ChannelPulse);
				}
				if (HAL_GetTick()%50 == 0 && flag == 0)    // 每50毫秒读取一次电流、电压
				{
					flag = 1;
					int32_t current = get_curr_val();

				#if 0//defined(PID_ASSISTANT_EN)
					set_computer_value(SEED_FACT_CMD, CURVES_CH1, &current, 1);
				#else
					printf("电源电压：%.2fV，电流：%dmA\r\n", get_vbus_val(), current); 
				#endif
					
					if (current > CURR_MAX)    // 判断是不是超过限定的值
					{
						if (curr_max_count++ > 5)    // 连续5次超过
						{
							LED2_ON;
							set_motor_disable();
							curr_max_count = 0;
							printf("电流超过限制！请检查原因，复位开发板在试！\r\n");
							while(1);
						}
					}
					
				}
				else if (HAL_GetTick()%50 != 0 && flag == 1)
				{
					flag = 0;
				}
			}
			
    }

#if 0//按键数量少,换向\禁用电机功能不开放
    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      /* 禁用电机 */
      set_motor_disable();
    }
    /* 扫描KEY5 */
    if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
    {
      /* 转换方向 */
      set_motor_direction( (++dir % 2) ? MOTOR_FWD : MOTOR_REV);
    }
#endif
    if (HAL_GetTick()%50 == 0 && flag == 0)    // 每50毫秒读取一次电流、电压
    {
      flag = 1;
      int32_t current = get_curr_val();

    #if 0//defined(PID_ASSISTANT_EN)
      set_computer_value(SEED_FACT_CMD, CURVES_CH1, &current, 1);
    #else
      printf("电源电压：%.2fV，电流：%dmA\r\n", get_vbus_val(), current); 
    #endif
      
      if (current > CURR_MAX)    // 判断是不是超过限定的值
      {
        if (curr_max_count++ > 5)    // 连续5次超过
        {
          LED2_ON;
          set_motor_disable();
          curr_max_count = 0;
          printf("电流超过限制！请检查原因，复位开发板在试！\r\n");
          while(1);
        }
      }
      
    }
    else if (HAL_GetTick()%50 != 0 && flag == 1)
    {
      flag = 0;
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
