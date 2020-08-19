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
#include "./key/bsp_key.h" 
#include "./led/bsp_led.h"
#include "./Encoder/bsp_encoder.h"
#include "./pid/bsp_pid.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"



extern _pid move_pid,speed_pid;
extern int pid_status;
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{

	/* ��ʼ��ϵͳʱ��Ϊ180MHz */
	SystemClock_Config();
	/*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
	DEBUG_USART_Config();
	printf("��ӭʹ��Ұ�� ��������� ������� �ٶȱջ����� λ��ʽPID����\r\n");
	printf("���°���1����Ŀ��ֵ������2����Ŀ��ֵ\r\n");	
  printf("����������ʹ��PID��������\r\n");	
  /* ��ʼ��ʱ��� */
  HAL_InitTick(5);
	/*�����жϳ�ʼ��*/
	Key_GPIO_Config();	
	/*led��ʼ��*/
	LED_GPIO_Config();
  /* ��ʼ��������ʱ����ʱ��20ms����һ���ж� */
	TIMx_Configuration();
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
  /*���������ʼ��*/
	stepper_Init();
  /* �ϵ�Ĭ��ֹͣ��� */
  Set_Stepper_Stop();
  /* PID�㷨������ʼ�� */
  PID_param_init();	

  /* Ŀ��λ��ת��Ϊ����������������ΪpidĿ��ֵ */
  move_pid.target_val = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
	int32_t Temp = TARGET_DISP * ENCODER_TOTAL_RESOLUTION;
	Set_Stepper_Start();
#if PID_ASSISTANT_EN
	set_computer_value(SEED_START_CMD, CURVES_CH1, NULL, 0);// ͬ����λ����������ť״̬
  set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &Temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
#endif

	while(1)
	{
    /* ɨ��KEY1������Ŀ��λ��*/
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
		{
      /* Ŀ��λ������48000����Ӧ���λ������20Ȧ */
      move_pid.target_val += 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
    /* ɨ��KEY2����СĿ��λ�� */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
      /* Ŀ��λ�ü�С48000����Ӧ���λ�ü���20Ȧ */
      move_pid.target_val -= 48000;
      
    #if PID_ASSISTANT_EN
      int temp = move_pid.target_val;
      set_computer_value(SEED_TARGET_CMD, CURVES_CH1, &temp, 1);// ��ͨ�� 1 ����Ŀ��ֵ
    #endif
		}
	}
} 	

/**
  * @brief  ��ʱ�������¼��ص�����
  * @param  ��
  * @retval ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* �жϴ����жϵĶ�ʱ�� */
  if(htim->Instance == BASIC_TIM)
  {
    Stepper_Ctrl();
  }
  else if(htim->Instance == ENCODER_TIM)
  {  
    /* �жϵ�ǰ�������� */
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      /* ���� */
      encoder_overflow_count--;
    else
      /* ���� */
      encoder_overflow_count++;
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
