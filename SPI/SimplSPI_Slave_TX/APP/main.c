#include "SWM320.h"


uint16_t *SPI1TXBuff = 0;
volatile uint32_t SPI1TXCount = 0;
volatile uint32_t SPI1TXIndex = 0;


void SerialInit(void);
void SPIMstInit(void);
void SPISlvInit(void);
void SPISlvSend(uint16_t buff[], uint32_t cnt);

int main(void)
{
	uint32_t i, j;
	uint16_t rxbuff[16] = {0};
	uint16_t txbuff[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	
	SystemInit();
	
	SerialInit();
	
	SPIMstInit();
	
	SPISlvInit();

	SPISlvSend(txbuff, 16);
	
	while(1==1)
	{
		GPIO_ClrBit(GPIOP, PIN12);	//SPI0_CS_Low();
		for(i = 0; i < 1200; i++);	// CS拉低后需要延时一下再发送
		for(i = 0, j = 0; i < 16; i++)
		{
			SPI0->DATA = 0xFF;
			while((SPI0->STAT & SPI_STAT_TFNF_Msk) == 0);
			
			if(SPI0->STAT & SPI_STAT_RFNE_Msk) rxbuff[j++] = SPI0->DATA;
		}
		while(j < 16)
		{
			if(SPI0->STAT & SPI_STAT_RFNE_Msk) rxbuff[j++] = SPI0->DATA;
		}
		GPIO_SetBit(GPIOP, PIN12);	//SPI0_CS_High();
		
		for(i = 0; i < 16; i++) printf("%d, ", rxbuff[i]);
		printf("\r\n\r\n");
		
		for(i = 0; i < SystemCoreClock/4; i++);
	}
}


void SPIMstInit(void)
{
	SPI_InitStructure SPI_initStruct;
	
	PORT_Init(PORTP, PIN9,  FUNMUX1_SPI0_SCLK, 0);
	PORT_Init(PORTP, PIN10, FUNMUX0_SPI0_MOSI, 0);
	PORT_Init(PORTP, PIN11, FUNMUX1_SPI0_MISO, 1);
	GPIO_Init(GPIOP, PIN12, 1, 0, 0);
#define SPI0_CS_Low()	GPIO_ClrBit(GPIOP, PIN12)
#define SPI0_CS_High()	GPIO_SetBit(GPIOP, PIN12)
	SPI0_CS_High();
	
	SPI_initStruct.clkDiv = SPI_CLKDIV_32;
	SPI_initStruct.FrameFormat = SPI_FORMAT_SPI;
	SPI_initStruct.SampleEdge = SPI_SECOND_EDGE;
	SPI_initStruct.IdleLevel = SPI_HIGH_LEVEL;
	SPI_initStruct.WordSize = 8;
	SPI_initStruct.Master = 1;
	SPI_initStruct.RXHFullIEn = 0;
	SPI_initStruct.TXEmptyIEn = 0;
	SPI_initStruct.TXCompleteIEn = 0;
	SPI_Init(SPI0, &SPI_initStruct);
	SPI_Open(SPI0);
}


void SPISlvInit(void)
{
	SPI_InitStructure SPI_initStruct;
	
	PORT_Init(PORTA, PIN9,  FUNMUX1_SPI1_SCLK, 1);
	PORT_Init(PORTA, PIN10, FUNMUX0_SPI1_MOSI, 1);
	PORT_Init(PORTA, PIN11, FUNMUX1_SPI1_MISO, 0);
	PORT_Init(PORTA, PIN12, FUNMUX0_SPI1_SSEL, 1);
	
	SPI_initStruct.clkDiv = 1;
	SPI_initStruct.FrameFormat = SPI_FORMAT_SPI;
	SPI_initStruct.SampleEdge = SPI_SECOND_EDGE;
	SPI_initStruct.IdleLevel = SPI_HIGH_LEVEL;
	SPI_initStruct.WordSize = 8;
	SPI_initStruct.Master = 0;
	SPI_initStruct.RXHFullIEn = 0;
	SPI_initStruct.TXEmptyIEn = 0;
	SPI_initStruct.TXCompleteIEn = 0;
	SPI_Init(SPI1, &SPI_initStruct);	
	SPI_Open(SPI1);
	
	SPI_INTEn(SPI1, SPI_IT_TX_EMPTY | (1 << 8));
	NVIC_EnableIRQ(SPI1_IRQn);
	
	/* 与PA12引脚相连，用于检测SPI从机的CS Assert事件 */
	GPIO_Init(GPIOA, PIN4, 0, 1, 0);			//输入，上拉使能
	
	EXTI_Init(GPIOA, PIN4, EXTI_FALL_EDGE);		//下降沿触发中断
	NVIC_EnableIRQ(GPIOA4_IRQn);	
	EXTI_Open(GPIOA, PIN4);
}

void SPISlvSend(uint16_t buff[], uint32_t cnt)
{
	SPI1TXBuff = buff;
	SPI1TXCount = cnt;
}

void SPI1_Handler(void)
{
	if((SPI1->IF & SPI_IF_TFE_Msk) || (SPI1->IF & SPI_IF_WTC_Msk))
	{
		while(SPI1->STAT & SPI_STAT_TFNF_Msk)
		{
			if(SPI1TXIndex < SPI1TXCount)
			{
				SPI1->DATA = SPI1TXBuff[SPI1TXIndex++];
			}
			else
			{
				SPI1->DATA = 0xFF;	// 没数据了，发送0xFF
			}
		}
		
		SPI1->IF = (1 << SPI_IF_TFE_Pos) | (1 << SPI_IF_WTC_Pos);
	}
}

void GPIOA4_Handler(void)
{	
	EXTI_Clear(GPIOA, PIN4);
	
	SPI1TXIndex = 0;
	
	SPI1->CTRL |=  (1 << SPI_CTRL_TFCLR_Pos);	// 发送FIFO清空
	SPI1->CTRL &= ~(1 << SPI_CTRL_TFCLR_Pos);
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN2, FUNMUX0_UART0_RXD, 1);	//GPIOA.2配置为UART0输入引脚
	PORT_Init(PORTA, PIN3, FUNMUX1_UART0_TXD, 0);	//GPIOA.3配置为UART0输出引脚
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

/****************************************************************************************************************************************** 
* 函数名称: fputc()
* 功能说明: printf()使用此函数完成实际的串口打印动作
* 输    入: int ch		要打印的字符
*			FILE *f		文件句柄
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
