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
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f4xx.h"
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h"
#include "./stepper/bsp_stepper_init.h"
#include "./key/bsp_key.h"
#include "./led/bsp_led.h"
#include "./Encoder/bsp_encoder.h"

/* 电机旋转方向 */
__IO int8_t motor_direction = 0;
/* 当前时刻总计数值 */
__IO int32_t capture_count = 0;
/* 上一时刻总计数值 */
__IO int32_t last_count = 0;
/* 单位时间内总计数值 */
__IO int32_t count_per_unit = 0;
/* 电机转轴转速 */
__IO float shaft_speed = 0.0f;
/* 累积圈数 */
__IO float number_of_rotations = 0.0f;


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
  int i = 0;
  
	/* 初始化系统时钟为180MHz */
	SystemClock_Config();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	printf("欢迎使用野火 电机开发板 步进电机 编码器测速 例程\r\n");
	printf("按下按键1启动电机、按键2停止、按键3改变方向\r\n");	
  /* 初始化时间戳 */
  HAL_InitTick(5);
	/*按键初始化*/
	Key_GPIO_Config();	
	/*led初始化*/
	LED_GPIO_Config();
	/*步进电机初始化*/
	stepper_Init();
  /* 上电默认停止电机，按键1启动 */
  MOTOR_EN(LOW);
  /* 编码器接口初始化 */
	Encoder_Init();
  
	while(1)
	{
    /* 扫描KEY1，启动电机 */
    if(Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
    {
      MOTOR_EN_TOGGLE;
    }
    /* 扫描KEY3，改变方向 */
    if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON)
    {
      static int j = 0;
      j > 0 ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
      j=!j;
    }
    
    /* 20ms计算一次 */
    /* 电机旋转方向 = 计数器计数方向 */
    motor_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * ENCODER_TIM_PERIOD  */
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* 单位时间内总计数值 = 当前时刻总计数值 - 上一时刻总计数值 */
    count_per_unit = capture_count - last_count;
    
    /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率 * 时间系数  */
    shaft_speed = (float)count_per_unit / ENCODER_TOTAL_RESOLUTION * 50 ;
    
    /* 累积圈数 = 当前时刻总计数值 / 编码器总分辨率  */
    number_of_rotations = (float)capture_count / ENCODER_TOTAL_RESOLUTION;

    /* 记录当前总计数值，供下一时刻计算使用 */
    last_count = capture_count;
    
    if(i == 50)/* 1s报告一次 */
    {
      printf("\r\n电机方向：%d\r\n", motor_direction);
      printf("单位时间内有效计数值：%d\r\n", (count_per_unit<0 ? abs(count_per_unit) : count_per_unit));
      printf("步进电机转速：%.2f 转/秒\r\n", shaft_speed);
      printf("累计圈数：%.2f 圈\r\n", number_of_rotations);
      i = 0;
    }
    delay_ms(20);
    i++;
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
