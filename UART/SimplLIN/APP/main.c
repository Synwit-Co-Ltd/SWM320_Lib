#include <string.h>
#include "SWM320.h"

/* 注意事项：
 * 1、LIN Break的宽度需要≥12.6个位时间长度
 * 2、LIN Break的宽度≥13.6个位时间长度时会触发2次break中断
 * 3、LIN Break的宽度≥14.6个位时间长度时会触发3次break中断
 * 4、依此类推，对于多出来的break中断，需要软件处理，比如根据两个break之间是否接收到数据判断break是否是多余的
 */

char TX_Str1[] = "Hi from Synwit\r\n";
char TX_Str2[] = "SWM320 LIN Demo\r\n";

#define UART_RX_LEN	 128
char UART_RXBuffer[UART_RX_LEN] = {0};
volatile uint32_t UART_RXIndex = 0;


void SerialInit(void);
void Serial1Init(void);
void LIN_FrameSend(char data[], uint32_t count);

int main(void)
{
	int i;
	
	SystemInit();
	
	SerialInit();
	
	Serial1Init();
	
	GPIO_Init(GPIOA, PIN9, 1, 0, 0);
   	
	while(1==1)
	{
		LIN_FrameSend(TX_Str1, sizeof(TX_Str1));
		for(i = 0; i < SystemCoreClock/8; i++) __NOP();
		
		LIN_FrameSend(TX_Str2, sizeof(TX_Str2));
		for(i = 0; i < SystemCoreClock/8; i++) __NOP();
	}
}


void Serial1Init(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN4, FUNMUX0_UART1_RXD, 1);
	PORT_Init(PORTA, PIN5, FUNMUX1_UART1_TXD, 0);
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThreshold = 3;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThreshold = 3;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutTime = 10;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART1, &UART_initStruct);
	UART_Open(UART1);
}


void LIN_FrameSend(char data[], uint32_t count)
{
	uint32_t i;
	
	UART_LINGenerate(UART1);
	while(UART1->LINCR & UART_LINCR_GENBRK_Msk) __NOP();	//完成自动清零
	
	for(i = 0; i < count; i++)
	{
		while(UART_IsTXFIFOFull(UART1)) __NOP();
		
		UART_WriteByte(UART1, data[i]);
	}
}


void UART0_Handler(void)
{
	uint32_t chr;
	
	if(UART_INTStat(UART0, UART_IT_RX_THR | UART_IT_RX_TOUT))
	{
		while(UART_IsRXFIFOEmpty(UART0) == 0)
		{
			if(UART_ReadByte(UART0, &chr) == 0)
			{
				if(UART_RXIndex < UART_RX_LEN)
				{
					UART_RXBuffer[UART_RXIndex] = chr;
					
					UART_RXIndex++;
				}
			}
		}
	}
	else if(UART_LINIsDetected(UART0))
	{
		UART0->LINCR |= UART_LINCR_BRKDETIF_Msk;	//清除中断标志
		
		GPIO_InvBit(GPIOA, PIN9);
		
		if(UART_RXIndex)
		{
			printf("LIN frame data:\r\n   %s\r\n", UART_RXBuffer);
			memset(UART_RXBuffer, 0, sizeof(UART_RXBuffer));
			UART_RXIndex = 0;
		}
	}
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
	UART_initStruct.RXThreshold = 3;
	UART_initStruct.RXThresholdIEn = 1;
	UART_initStruct.TXThreshold = 3;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutTime = 10;		//10个字符时间内未接收到新的数据则触发超时中断
	UART_initStruct.TimeoutIEn = 1;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
	
	UART_LINConfig(UART0, 1, 0);
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
