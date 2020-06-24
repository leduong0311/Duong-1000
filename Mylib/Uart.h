/* 
June 23,2020
LV_Embedded
 */
#ifndef UART_H
#define UART_H

#include "../Src/stm32f4xx.h"

#define USARTx                  USART2
#define USARTx_CLK              RCC_APB1Periph_USART2
#define USARTx_TX_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define USARTx_TX_PIN           GPIO_Pin_2
#define USARTx_TX_GPIO_PORT     GPIOA
#define USARTx_TX_SOURCE        GPIO_PinSource2
#define USARTx_TX_AF            GPIO_AF_USART2

#define USARTx_RX_GPIO_CLK      RCC_APB1Periph_USART2
#define USARTx_RX_PIN           GPIO_Pin_3
#define USARTx_RX_GPIO_PORT     GPIOA
#define USARTx_RX_SOURCE        GPIO_PinSource3
#define USARTx_RX_AF            GPIO_AF_USART2

uint16_t USART_GetChar();
void USART_PutChar(char c);
void USART_PutString(char *s);
void USART_Config(void);

#endif