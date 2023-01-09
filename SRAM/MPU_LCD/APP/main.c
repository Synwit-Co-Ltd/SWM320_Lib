#include "SWM320.h"

#include "ugui.h"
#include "NT35510.h"

#define LCD_HDOT	480
#define LCD_VDOT	800

UG_GUI gui;

void SerialInit(void);

int main(void)
{		
	uint32_t i, j;
	volatile uint16_t temp;
	uint16_t color[5] = {C_RED, C_GREEN, C_BLUE, C_PERU, 0xFFFF};

	SystemInit();
	
	SerialInit();
	
	TFTInit();

	TFTClear(C_RED);
	
	UG_Init(&gui,(void(*)(UG_S16,UG_S16,UG_COLOR))TFTDrawPoint, LCD_HDOT, LCD_VDOT);
	
	UG_DrawLine(40, 50, 240, 50, C_GREEN);
	
	UG_DrawLine(40, 36, 40, 236, C_BLUE);
		
	UG_DrawCircle(200, 210, 50, C_GREEN);
	UG_DrawCircle(200, 210, 51, C_GREEN);
	UG_DrawCircle(200, 210, 52, C_GREEN);

	while(1==1)
	{		
		for(i = 0; i < 5; i++)
		{
			for(j = 0; j < SystemCoreClock/20; j++) __NOP();
			
			UG_FillFrame(60, 75, 200, 150, color[i]);
			
			temp = TFTReadPoint(65, 80);
			temp = 0;
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
