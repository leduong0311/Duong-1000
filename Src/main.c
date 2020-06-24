#include "stm32f4xx.h"                  // Device header
#include "DHT12.h"
#include "Systick.h"
#include <stdlib.h>
//#include "system_timetick.h"

#define		BUFF_SIZES			9

uint8_t txbuff[];
//uint8_t temp=0,temp1=0,hum=0,hum1=0;
float temp=0,hum =0;
int temp1 = 0; hum1 = 0;
uint8_t count=0;
uint8_t buffer[4];
void init_main(void);
int main()
{
    
		Systick_Configuration();
	//SysTick_Config(SystemCoreClock/100);
    I2C_Config();
		init_main();
    while (1)
    {
        I2C_Read(I2Cxx,SLAVE_ADDRESS,0x00,buffer,4);
				SystickDelay_ms(1000);
			   hum   = (float) ((int) buffer[0]) + ((float) buffer[1]/10);
		     temp  = (float) ((int) buffer[2]) + ((float) buffer[3]/10);
			   hum1 = (int) buffer[0]; 
			temp1 = (int) buffer[2];
			txbuff[0] = hum1/10 + 48;
			txbuff[1] = hum1 - (hum1/10)*10 + 48;
			txbuff[2] = '.';
			txbuff[3] = buffer[3] + 48;
			txbuff[4] = '&';
			txbuff[5] = temp1/10 + 48;
			txbuff[6] = temp1 - (temp1/10)*10 + 48;
			txbuff[7] = '.';
			txbuff[8] = buffer[1] + 48;
			//txbuff[0] = 
			
			//txbuff[0] = hum;
		  //txbuff[1] = temp;
     //   if(tick_count == 100){
			 //   tick_count = 0;
			    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
			    DMA1_Stream4->NDTR = BUFF_SIZES;
			    DMA_Cmd(DMA1_Stream4, ENABLE);
		//}	
				count++;
    }
}

void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
	DMA_InitTypeDef  	DMA_InitStructure;
   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 

  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UART4, &USART_InitStructure);
	
	/* Enable USART */
  USART_Cmd(UART4, ENABLE);
	
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
	
	
	/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) txbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZES;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream4, ENABLE);

}
