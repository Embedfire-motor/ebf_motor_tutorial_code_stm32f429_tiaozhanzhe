#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include "./protocol/protocol.h"
#include <stdio.h>

//串口波特率
#define DEBUG_USART_BAUDRATE                    115200

//#define RS232
#define USB_TO_UART

#ifdef RS232
  //引脚定义
  /*******************************************************/
  #define DEBUG_USART                             USART3
  #define DEBUG_USART_CLK_ENABLE()                __USART3_CLK_ENABLE();

  #define RCC_PERIPHCLK_UARTx                     RCC_PERIPHCLK_USART3
  #define RCC_UARTxCLKSOURCE_SYSCLK               RCC_USART3CLKSOURCE_SYSCLK

  #define DEBUG_USART_RX_GPIO_PORT                GPIOB
  #define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
  #define DEBUG_USART_RX_PIN                      GPIO_PIN_11
  #define DEBUG_USART_RX_AF                       GPIO_AF7_USART3

  #define DEBUG_USART_TX_GPIO_PORT                GPIOB
  #define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
  #define DEBUG_USART_TX_PIN                      GPIO_PIN_10
  #define DEBUG_USART_TX_AF                       GPIO_AF7_USART3

  #define DEBUG_USART_IRQHandler                  USART3_IRQHandler
  #define DEBUG_USART_IRQ                 		    USART3_IRQn
  /************************************************************/
#else
  //引脚定义
  /*******************************************************/
  #define DEBUG_USART                             USART1
  #define DEBUG_USART_CLK_ENABLE()                __USART1_CLK_ENABLE();

  #define RCC_PERIPHCLK_UARTx                     RCC_PERIPHCLK_USART1
  #define RCC_UARTxCLKSOURCE_SYSCLK               RCC_USART1CLKSOURCE_SYSCLK

  #define DEBUG_USART_RX_GPIO_PORT                GPIOA
  #define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
  #define DEBUG_USART_RX_PIN                      GPIO_PIN_9
  #define DEBUG_USART_RX_AF                       GPIO_AF7_USART1

  #define DEBUG_USART_TX_GPIO_PORT                GPIOA
  #define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
  #define DEBUG_USART_TX_PIN                      GPIO_PIN_10
  #define DEBUG_USART_TX_AF                       GPIO_AF7_USART1

  #define DEBUG_USART_IRQHandler                  USART1_IRQHandler
  #define DEBUG_USART_IRQ                 		    USART1_IRQn
  /************************************************************/
#endif

void uart_FlushRxBuffer(void);
void Usart_SendByte(uint8_t str);
void Usart_SendString(uint8_t *str);
void DEBUG_USART_Config(void);
//int fputc(int ch, FILE *f);
extern UART_HandleTypeDef UartHandle;

#endif /* __USART1_H */
