#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx_hal.h"

/*宏定义*/
/*******************************************************/
//宏定义对应开发板的接口 1 、2 、3 、4
#define CHANNEL_SW 1

#if(CHANNEL_SW == 1)
//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_7   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_0
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor 脉冲
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
//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_12   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor 脉冲
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
//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_15   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_9
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor 脉冲
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
//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_8   
#define MOTOR_DIR_GPIO_PORT            	GPIOD                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOD_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOD                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOD_CLK_ENABLE()
	
//Motor 脉冲
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


/*频率相关参数*/
//定时器实际时钟频率为：168MHz/TIM_PRESCALER
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//168/TIM_PRESCALER = 28MHz
//具体需要的频率可以自己计算
#define TIM_PRESCALER                6
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	      //高电平
#define LOW  GPIO_PIN_RESET	    //低电平

#define ON  HIGH	              //开
#define OFF LOW	                //关

#define CW 	LOW		              //顺时针
#define CCW HIGH      	        //逆时针

//控制使能引脚
/* 带参宏，可以像内联函数一样使用 */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

#define MOTOR_EN_TOGGLE     HAL_GPIO_TogglePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN)
#define MOTOR_PUL_TOGGLE    HAL_GPIO_TogglePin(MOTOR_PUL_PORT,MOTOR_PUL_PIN)

extern TIM_HandleTypeDef TIM_StepperHandle;

extern void stepper_Init(void);

#endif /* __STEP_MOTOR_INIT_H */
