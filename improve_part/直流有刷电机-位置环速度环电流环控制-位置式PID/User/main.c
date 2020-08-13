/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   ֱ����ˢ���-λ�û�����-����ʽPID
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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
#include "./Encoder/bsp_encoder.h"
#include "./tim/bsp_basic_tim.h"
#include "./pid/bsp_pid.h"
#include "./adc/bsp_adc.h"
#include "./protocol/protocol.h"

int pulse_num=0;
	
void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}	
	
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
  int32_t target_location = PER_CYCLE_PULSES*30;
  
  /* HAL ���ʼ�� */
  HAL_Init();
  
	/* ��ʼ��ϵͳʱ��Ϊ168MHz */
	SystemClock_Config();
  
	/* ��ʼ������ GPIO */
	Key_GPIO_Config();
  
  /* ��ʼ�� LED */
  LED_GPIO_Config();
  
  /* Э���ʼ�� */
  protocol_init();
  
  /* ��ʼ������ */
  DEBUG_USART_Config();

  /* �����ʼ�� */
  motor_init();
  
	set_motor_disable();     // ֹͣ��� 

  /* ADC ��ʼ�� */
  ADC_Init();
  
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
  
  /* ��ʼ��������ʱ�������ڴ���ʱ���� */
  TIMx_Configuration();
  
  /* PID ������ʼ�� */
  PID_param_init();
  
  set_pid_target(&pid_location, target_location);    // ����Ŀ��ֵ
  
#if defined(PID_ASSISTANT_EN)
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // ͬ����λ����������ť״̬
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_location, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
#endif

	while(1)
	{
    /* �������ݴ��� */
    receiving_process();
    
    /* ɨ��KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
    #if defined(PID_ASSISTANT_EN) 
      set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // ͬ����λ����������ť״̬
    #endif
      set_pid_target(&pid_location, target_location);    // ����Ŀ��ֵ
      set_motor_enable();              // ʹ�ܵ��
			
			while(1)
			{
				/* �������ݴ��� */
				receiving_process();
				/* ɨ��KEY1 */
				if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
				{
					/* ����Ŀ��λ�� */
					target_location += PER_CYCLE_PULSES;
					
					set_pid_target(&pid_location, target_location);
				#if defined(PID_ASSISTANT_EN)
					set_computer_value(SEND_TARGET_CMD, CURVES_CH1,  &target_location, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
				#endif
				}

				/* ɨ��KEY2 */
				if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
				{
					/* ��СĿ��λ�� */
					target_location -= PER_CYCLE_PULSES;
					
					set_pid_target(&pid_location, target_location);
				#if defined(PID_ASSISTANT_EN)
					set_computer_value(SEND_TARGET_CMD, CURVES_CH1,  &target_location, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
				#endif
				}
			}
    }
#if 0//����������,����\���õ��������ʱ����,��ʹ��PID�������ֵ���
    /* ɨ��KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      set_motor_disable();     // ֹͣ���
      set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);               // ͬ����λ����������ť״̬
    }
#endif

	}
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
 void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {};
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
