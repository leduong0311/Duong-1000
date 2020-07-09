#include "stm32f4xx.h"                  // Device header
#include "DHT12.h"
#include "Systick.h"
#include <stdlib.h>
#include <math.h>
//#include "system_timetick.h"

#define		BUFF_SIZES			9
#define		BUFF_SIZE			4

uint8_t 	rxbuff[BUFF_SIZE];
uint8_t 	a[BUFF_SIZE];

uint8_t txbuff[9];
float ukk = 0;
//uint8_t temp=0,temp1=0,hum=0,hum1=0;
float temp=0,hum =0;
int temp1 = 0, hum1 = 0;
double count=0;
uint8_t buffer[4];
void init_main(void);

float ek = 0, ek_1 =0,ek_2=0,uk = 0, uk_1 = 0,setpoint = 38;
float kp = 9.9,ki= 1.2 , kd = 7.45 , T = 0.01;   
float PID_control(float measure){
ek_2 = ek_1;
ek_1 = ek;
ek = setpoint - measure;
uk_1 = uk;
uk = uk_1 +kp*(ek - ek_1) + ki*T/2*(ek + ek_1)+kd/T*(ek - 2*ek_1 +ek_2); 
if(uk > 5) uk = 5;
if(uk < 0) uk = 0;
return(uk);
}


int main()
{
    
		Systick_Configuration();
	//SysTick_Config(SystemCoreClock/100);
    I2C_Config();
		init_main();
	  TIM_OCInitTypeDef TIM_OCStruct;
		TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	
    while (1)
    {
			
		  /////////////////////////////////
			setpoint = (float)((a[0]-48)*10 + a[1] - 48 + (a[3] -48)/10);
///////////////////////////////////////////			
			
        I2C_Read(I2Cxx,SLAVE_ADDRESS,0x00,buffer,4);
				SystickDelay_ms(500);
			
			   hum   = (float) ((int) buffer[0]) + ((float) buffer[1]/10);
		     temp  = (float) (buffer[2]) + ((float) buffer[3]/10.0);
			   hum1 = (int) buffer[0]; 
			   temp1 = (int) buffer[2];
			//////////////////tach cac so dua vao buffer/////////////////
			txbuff[0] = hum1/10 + 48;
			txbuff[1] = hum1 - (hum1/10)*10 + 48;
			txbuff[2] = '.';
			txbuff[3] = buffer[3] + 48;
			txbuff[4] = '&';
			txbuff[5] = temp1/10 + 48;
			txbuff[6] = temp1 - (temp1/10)*10 + 48;
			txbuff[7] = '.';
			txbuff[8] = buffer[1] + 48;
		
			 ////////////uart dma/////////////////////
			if(count == 2){
				count = 0;
			    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
			    DMA1_Stream4->NDTR = BUFF_SIZES;
			    DMA_Cmd(DMA1_Stream4, ENABLE);
				 }
					///////////////////////xung pwm///////////////////
				 if(temp >= 37.5){
					TIM_OCStruct.TIM_Pulse = 0; /* 25% duty cycle */
       TIM_OC1Init(TIM4, &TIM_OCStruct);
       TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
					 }
				 if(temp < 38.5){
					TIM_OCStruct.TIM_Pulse = 9999; /* 25% duty cycle */
       TIM_OC1Init(TIM4, &TIM_OCStruct);
       TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
					 }
//			 TIM_OCStruct.TIM_Pulse = 9999; /* 25% duty cycle */
//       TIM_OC1Init(TIM4, &TIM_OCStruct);
//       TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
//			ukk = sqrt(PID_control(temp));
//			TIM_OCStruct.TIM_Pulse = 9999*ukk; /* 25% duty cycle */
//       TIM_OC1Init(TIM4, &TIM_OCStruct);
//       TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
				 
		//}	
				count ++;
    }
}

void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
	DMA_InitTypeDef  	DMA_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  TIM_OCInitTypeDef TIM_OCStruct;
	NVIC_InitTypeDef  NVIC_InitStructure;	
   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
	 GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		TIM_BaseStruct.TIM_Prescaler = 83;
		TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseStruct.TIM_Period = 9999; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
		/* Initialize TIM4 */
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    /* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);  
		
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


//////////////////////////////////////////
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
	
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream2, ENABLE);
		
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);

}
///////////////////bo dieu khien pid///////////////////////
void DMA1_Stream2_IRQHandler(void)
{
  uint16_t i;

  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);

  for(i=0; i<BUFF_SIZE; i++){
   // a[index + i] = rxbuff[i];
		a[i] = rxbuff[i];
//	index = index + BUFF_SIZE;
  //rcv_flag = 1;
	}
	DMA_Cmd(DMA1_Stream2, ENABLE);
}
