#include "GlobalVar.h"
#include "usart.h"
#include "stm32f10x_dma.h"
#include "string.h"

#define UART4_DMA_BUF_NUM 8

unsigned char receiveBuf[8];
unsigned char receiveCount;


static uint8_t uart4_rxdata[UART4_DMA_BUF_NUM]; 

USART_RECV_CALLBACK _usart_recv_callbacks[6] = { 0 };

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static void uart4_trx_dma_config(void)
{
	DMA_InitTypeDef DMA_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);  
	DMA_DeInit(DMA2_Channel3); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart4_rxdata; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize = UART4_DMA_BUF_NUM; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMA2_Channel3, &DMA_InitStructure); 

  DMA_ITConfig( DMA2_Channel3, DMA_IT_TC, ENABLE );
	DMA_ITConfig(DMA2_Channel3,DMA_IT_TE,ENABLE);

	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
  DMA_Cmd(DMA2_Channel3, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

uint32_t USART_Configuration(const struct USARTDefine* configs, uint16_t len) {
	uint16_t u = 0;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	
	for( u = 0; u < len; ++u) {
		struct USARTDefine config = configs[u];
		if( config.usartdef == UART5 ) {
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
			_usart_recv_callbacks[BOARD_USART_1] = config.recvcallback;
		} else if(config.usartdef == UART4 ) {
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
			_usart_recv_callbacks[BOARD_USART_2] = config.recvcallback;
			
		}
		GPIO_InitStructure.GPIO_Pin = config.pin_of_txd;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(config.pin_of_txd_group, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = config.pin_of_rxd;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(config.pin_of_rxd_group, &GPIO_InitStructure);

		USART_InitStructure.USART_BaudRate = config.bandrate;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(config.usartdef, &USART_InitStructure);
		
		if( config.recvcallback != 0 ) { 
			USART_ITConfig(config.usartdef,USART_IT_RXNE,ENABLE);
			if( config.usartdef == UART5 ) {
				NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			} else if(config.usartdef == UART4 ) {
				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			}
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);			
		}
		
		USART_Cmd(config.usartdef, ENABLE);	
		
		if(config.usartdef == UART4 ){
			//USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; 
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
			NVIC_Init(&NVIC_InitStructure);  
			USART_ClearFlag(UART4,USART_FLAG_IDLE);
		  USART_ClearFlag(UART4,USART_FLAG_TC);
			uart4_trx_dma_config();		
		}
	}
	return 0;
}


void _usart_nvic_config(uint32_t usart_irqn, uint32_t preemptionpriority,
		uint32_t subpriority) {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = usart_irqn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemptionpriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void _usart_comm_irqhandler(USART_TypeDef* usarttypedef,enum BOARD_USART_TYPE usart_type) {
	unsigned char recv_char;
	if (USART_GetFlagStatus(usarttypedef, USART_FLAG_RXNE) != RESET) {
		USART_ClearITPendingBit(usarttypedef, USART_IT_RXNE);
		recv_char = USART_ReceiveData(usarttypedef);
		if (_usart_recv_callbacks[usart_type] != NULL)
			_usart_recv_callbacks[usart_type](usart_type, recv_char);
	}
}

void USART1_IRQHandler(void) {
	_usart_comm_irqhandler(USART1, BOARD_USART_1);
}

void USART2_IRQHandler(void) {
	_usart_comm_irqhandler(USART2, BOARD_USART_2);
}

void _usart_sendchar(USART_TypeDef* usart_typedef, uint8_t sChar) {
	USART_SendData(usart_typedef, (uint8_t) sChar);
	while (USART_GetFlagStatus(usart_typedef, USART_FLAG_TXE) == RESET) {
	}
}

void _usart_sendstring(USART_TypeDef* usart_typedef, uint8_t* string, uint8_t length) {
	uint8_t* p = string;
	uint8_t len;
	
	if(length==0){
		while( *p++ != '\0' ){
			USART_SendData(usart_typedef, (uint8_t)(*p));
			while (USART_GetFlagStatus(usart_typedef, USART_FLAG_TXE) == RESET) {
			}
		}
	}
	else
	{
		for(len=0;len<length;len++){
			USART_SendData(usart_typedef, (uint8_t)(*p));
			p++;
			while (USART_GetFlagStatus(usart_typedef, USART_FLAG_TXE) == RESET) {
			}
		}
	}
}
PUTCHAR_PROTOTYPE {
	USART_SendData(USART1, (u8) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	return ch;
}


/*************************************************************
 ´®¿ÚÖÐ¶Ï idleÖÐ¶Ï
*************************************************************/
void UART4_IRQHandler(void)
{	
	uint16_t dma_len;
	uint32_t i;
	uint16_t dmadatalength;

	if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)
	{
		DMA_Cmd(DMA2_Channel3,DISABLE);
		dma_len = DMA_GetCurrDataCounter(DMA2_Channel3);
		dmadatalength = UART4_DMA_BUF_NUM - dma_len;
		DMA_ClearFlag(DMA2_FLAG_GL3 | DMA2_FLAG_TC3 | DMA2_FLAG_TE3 | DMA2_FLAG_HT3);//
		DMA_SetCurrDataCounter(DMA2_Channel3,UART4_DMA_BUF_NUM);
		DMA_Cmd(DMA2_Channel3,ENABLE);
		usart_dma_dataprocess(uart4_rxdata,dmadatalength);
	}
	i = UART4->SR;
	i = UART4->DR;
}
// uart4 rx
void DMA2_Channel3_IRQHandler(void)	
{
	DMA_ClearITPendingBit(DMA2_IT_TC3);
	DMA_ClearITPendingBit(DMA2_IT_TE3);
	DMA_Cmd(DMA2_Channel3,DISABLE);
	DMA_ClearFlag(DMA2_FLAG_GL3 | DMA2_FLAG_TC3 | DMA2_FLAG_TE3 | DMA2_FLAG_HT3);
	DMA_SetCurrDataCounter(DMA2_Channel3,UART4_DMA_BUF_NUM);
	DMA_Cmd(DMA2_Channel3,ENABLE);
	usart_dma_dataprocess(uart4_rxdata,UART4_DMA_BUF_NUM);
}
// uart4 tx 
void DMA2_Channel5_IRQHandler(void)	
{
	if(DMA_GetITStatus(DMA2_IT_TC5)){
		
		DMA_ClearITPendingBit(DMA2_IT_GL5);
	}
}

void uart_dma_sendNbytes(uint8_t *buffer,uint16_t buffersize)
{
	DMA_InitTypeDef DMA_InitStructure; 
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);  
	DMA_DeInit(DMA2_Channel5); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR); 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = buffersize; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMA2_Channel5, &DMA_InitStructure); 

  DMA_ITConfig( DMA2_Channel5, DMA_IT_TC, ENABLE );
	DMA_ITConfig(DMA2_Channel5,DMA_IT_TE,ENABLE);

	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
  DMA_Cmd(DMA2_Channel5, ENABLE);
	
}
void usart_dma_dataprocess(uint8_t *data,uint16_t datalen){
	uart_dma_sendNbytes(data,datalen);
}

