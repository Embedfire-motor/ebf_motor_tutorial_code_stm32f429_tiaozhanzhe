#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx_hal.h"

/*�궨��*/
/*******************************************************/
//�궨���Ӧ������Ľӿ� 1 ��2 ��3 ��4
#define CHANNEL_SW 1

#if(CHANNEL_SW == 1)
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_7   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_0
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM1_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM1_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM1
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM1_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOE
#define MOTOR_PUL_PIN             		  GPIO_PIN_9
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF1_TIM1
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1

#define MOTOR_TIM_IT_CCx                TIM_IT_CC1
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC1
#elif(CHANNEL_SW == 2)
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_12   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM1_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM1_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM1
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM1_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOE
#define MOTOR_PUL_PIN             		  GPIO_PIN_11
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF1_TIM1
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_2

#define MOTOR_TIM_IT_CCx                TIM_IT_CC2
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC2
#elif(CHANNEL_SW == 3)
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_15   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_9
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM1_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM1_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM1
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM1_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOE
#define MOTOR_PUL_PIN             		  GPIO_PIN_13
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF1_TIM1
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_3

#define MOTOR_TIM_IT_CCx                TIM_IT_CC3
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC3
#elif(CHANNEL_SW == 4)
//Motor ���� 
#define MOTOR_DIR_PIN                  	GPIO_PIN_8   
#define MOTOR_DIR_GPIO_PORT            	GPIOD                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOD_CLK_ENABLE()

//Motor ʹ�� 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor ����
#define MOTOR_PUL_IRQn                  TIM1_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM1_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM1
#define MOTOR_PUL_CLK_ENABLE()  		    __TIM1_CLK_ENABLE()

#define MOTOR_PUL_PORT       			      GPIOE
#define MOTOR_PUL_PIN             		  GPIO_PIN_14
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOE_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF1_TIM1
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_4

#define MOTOR_TIM_IT_CCx                TIM_IT_CC4
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC4
#endif


/*Ƶ����ز���*/
//��ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/TIM_PRESCALER
//���� �߼���ʱ���� Ƶ��Ϊ168MHz,������ʱ��Ϊ84MHz
//168/TIM_PRESCALER = 28MHz
//������Ҫ��Ƶ�ʿ����Լ�����
#define TIM_PRESCALER                6
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	      //�ߵ�ƽ
#define LOW  GPIO_PIN_RESET	    //�͵�ƽ

#define ON  HIGH	              //��
#define OFF LOW	                //��

#define CW 	LOW		              //˳ʱ��
#define CCW HIGH      	        //��ʱ��

//����ʹ������
/* ���κ꣬��������������һ��ʹ�� */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

#define MOTOR_EN_TOGGLE     HAL_GPIO_TogglePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN)
#define MOTOR_PUL_TOGGLE    HAL_GPIO_TogglePin(MOTOR_PUL_PORT,MOTOR_PUL_PIN)

extern TIM_HandleTypeDef TIM_StepperHandle;

extern void stepper_Init(void);

#endif /* __STEP_MOTOR_INIT_H */
