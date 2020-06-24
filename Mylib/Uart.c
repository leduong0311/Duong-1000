/* 
June 23,2020
LV_Embedded
 */

#include "Uart.h"

void USART_Config(void)
{
    /* enable clk gpio and uart */
    RCC_AHB1PeriphClockCmd(USARTx_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(USARTx_TX_GPIO_CLK,ENABLE);
    RCC_APB1PeriphClockCmd(USARTx_RX_GPIO_CLK,ENABLE);

    /* GPIO Configuration */
    /* TX */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=USARTx_TX_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(USARTx_TX_GPIO_PORT,&GPIO_InitStructure);

    /* RX */
    GPIO_InitStructure.GPIO_Pin=USARTx_RX_PIN;
    GPIO_Init(USARTx_RX_GPIO_PORT,&GPIO_InitStructure);

    /* GPIO AF Configuration */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT,USARTx_TX_SOURCE,USARTx_TX_AF);
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT,USARTx_RX_SOURCE,USARTx_RX_AF);

    /* USART Configuration */
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_WordLength=USART_WordLength_8b;
    USART_InitStructure.USART_StopBits=USART_StopBits_1;
    USART_InitStructure.USART_Parity=USART_Parity_No;
    USART_InitStructure.USART_BaudRate=9600;
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    
    USART_Init(USARTx,&USART_InitStructure);
    USART_Cmd(USARTx,ENABLE);
}

uint16_t USART_Getchar()
{
    /* waiting for receive data */
    while (!USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
        ;
    /* read data */
    return USART_ReceiveData(USART2);
}

void USART_PutChar(char c)
{
    /* waiting for trans data */
    while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
        ;
    /* Send data */
    USART_SendData(USART2, c);
}

void USART_PutString(char *s)
{
    while (*s)
    {
        USART_PutChar(*s++);
    }
}

