/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   GPIO���--ʹ�ù̼������LED��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://fire-stm32.taobao.com
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

/* �����ת���� */
__IO int8_t motor_direction = 0;
/* ��ǰʱ���ܼ���ֵ */
__IO int32_t capture_count = 0;
/* ��һʱ���ܼ���ֵ */
__IO int32_t last_count = 0;
/* ��λʱ�����ܼ���ֵ */
__IO int32_t count_per_unit = 0;
/* ���ת��ת�� */
__IO float shaft_speed = 0.0f;
/* �ۻ�Ȧ�� */
__IO float number_of_rotations = 0.0f;


/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
  int i = 0;
  
	/* ��ʼ��ϵͳʱ��Ϊ180MHz */
	SystemClock_Config();
	/*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
	DEBUG_USART_Config();
	printf("��ӭʹ��Ұ�� ��������� ������� ���������� ����\r\n");
	printf("���°���1�������������2ֹͣ������3�ı䷽��\r\n");	
  /* ��ʼ��ʱ��� */
  HAL_InitTick(5);
	/*������ʼ��*/
	Key_GPIO_Config();	
	/*led��ʼ��*/
	LED_GPIO_Config();
	/*���������ʼ��*/
	stepper_Init();
  /* �ϵ�Ĭ��ֹͣ���������1���� */
  MOTOR_EN(LOW);
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
  
	while(1)
	{
    /* ɨ��KEY1��������� */
    if(Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
    {
      MOTOR_EN_TOGGLE;
    }
    /* ɨ��KEY3���ı䷽�� */
    if(Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON)
    {
      static int j = 0;
      j > 0 ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
      j=!j;
    }
    
    /* 20ms����һ�� */
    /* �����ת���� = �������������� */
    motor_direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* ��ǰʱ���ܼ���ֵ = ������ֵ + ����������� * ENCODER_TIM_PERIOD  */
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* ��λʱ�����ܼ���ֵ = ��ǰʱ���ܼ���ֵ - ��һʱ���ܼ���ֵ */
    count_per_unit = capture_count - last_count;
    
    /* ת��ת�� = ��λʱ���ڵļ���ֵ / �������ֱܷ��� * ʱ��ϵ��  */
    shaft_speed = (float)count_per_unit / ENCODER_TOTAL_RESOLUTION * 50 ;
    
    /* �ۻ�Ȧ�� = ��ǰʱ���ܼ���ֵ / �������ֱܷ���  */
    number_of_rotations = (float)capture_count / ENCODER_TOTAL_RESOLUTION;

    /* ��¼��ǰ�ܼ���ֵ������һʱ�̼���ʹ�� */
    last_count = capture_count;
    
    if(i == 50)/* 1s����һ�� */
    {
      printf("\r\n�������%d\r\n", motor_direction);
      printf("��λʱ������Ч����ֵ��%d\r\n", (count_per_unit<0 ? abs(count_per_unit) : count_per_unit));
      printf("�������ת�٣�%.2f ת/��\r\n", shaft_speed);
      printf("�ۼ�Ȧ����%.2f Ȧ\r\n", number_of_rotations);
      i = 0;
    }
    delay_ms(20);
    i++;
	}
} 	


/**
  * @brief  ϵͳʱ������ 
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
  * @param  ��
  * @retval ��
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* ʹ��HSE������HSEΪPLL��ʱ��Դ������PLL�ĸ��ַ�Ƶ����M N P Q 
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

  /* ���� OverDrive ģʽ */
  HAL_PWREx_EnableOverDrive();
 
  /* ѡ��PLLCLK��ΪSYSCLK�������� HCLK, PCLK1 and PCLK2 ��ʱ�ӷ�Ƶ���� 
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
