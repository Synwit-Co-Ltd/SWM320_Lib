#include "SWM320.h"


char SPI1RXBuffer[128] = {0};
volatile uint32_t SPI1RXIndex = 0;
volatile uint32_t SPI1RXFinish = 0;


void SerialInit(void);
void SPIMstInit(void);
void SPISlvInit(void);

int main(void)
{
	SystemInit();
	
	SerialInit();
	
	SPIMstInit();
	
	SPISlvInit();
	
	TIMR_Init(TIMR0, TIMR_MODE_TIMER, SystemCoreClock/1000, 1);
	TIMR_Start(TIMR0);
	
	while(1==1)
	{
		while(SPI1RXFinish == 0) __NOP();
		SPI1RXFinish = 0;
		SPI1RXIndex = 0;
		
		printf("%s\n\n", SPI1RXBuffer);
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


void TIMR0_Handler(void)
{
	static int counter = 0;
	static char txbuffer[64] = "Hello from Synwit!\r\n";
	
	TIMR_INTClr(TIMR0);
	
	counter += 1;
	
	if(counter == 1)
	{
		SPI0_CS_Low();
	}
	else if(counter < 50)
	{
		SPI0->DATA = txbuffer[counter - 2];
	}
	else if(counter == 50)
	{
		SPI0_CS_High();
	}
	else if(counter == 1000)
	{
		counter = 0;
	}
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
	SPI_initStruct.RXHFullIEn = 1;
	SPI_initStruct.TXEmptyIEn = 0;
	SPI_initStruct.TXCompleteIEn = 0;
	SPI_Init(SPI1, &SPI_initStruct);
	
	SPI_INTEn(SPI1, SPI_IT_RX_OVF);	// 溢出后SPI不再接收数据，因此需要在溢出中断中清空RXFIFO
	
	SPI_Open(SPI1);
	
	/* 与PA12引脚相连，用于检测SPI从机的CS Deassert事件 */
	GPIO_Init(GPIOA, PIN4, 0, 1, 0);			//输入，上拉使能
	
	EXTI_Init(GPIOA, PIN4, EXTI_RISE_EDGE);		//上升沿触发中断
	NVIC_EnableIRQ(GPIOA4_IRQn);	
	EXTI_Open(GPIOA, PIN4);
}

void SPI1_Handler(void)
{
	if(SPI_INTStat(SPI1, SPI_IT_RX_OVF))
	{
		SPI1->CTRL |= SPI_CTRL_RFCLR_Msk;
		__NOP();__NOP();__NOP();__NOP();
		SPI1->CTRL &= ~SPI_CTRL_RFCLR_Msk;

		SPI_INTClr(SPI1, SPI_IT_RX_OVF);
	}
	
	if(SPI1->IF & SPI_IF_RFHF_Msk)
	{
		while(SPI1->STAT & SPI_STAT_RFNE_Msk)
		{
			SPI1RXBuffer[SPI1RXIndex++] = SPI1->DATA;
		}
		
		SPI1->IF = (1 << SPI_IF_RFHF_Pos);	// 先读出数据，再清除标志
	}
}

void GPIOA4_Handler(void)
{	
	EXTI_Clear(GPIOA, PIN4);
	
	while(SPI1->STAT & SPI_STAT_RFNE_Msk)
	{
		SPI1RXBuffer[SPI1RXIndex++] = SPI1->DATA;
	}
	
	SPI1RXFinish = 1;
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
