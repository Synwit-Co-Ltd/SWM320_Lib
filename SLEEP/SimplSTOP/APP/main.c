#include "SWM320.h"

/* 注意：
	1、进入STOP后只能通过Wake-up引脚唤醒，不能按复位按键复位芯片
	2、从STOP模式Wake-up后程序从头开始重新执行，但Backup区的数据可以保持
	3、Wake-up引脚是PB12，下降沿唤醒，休眠期间需要保持高电平
*/

void SerialInit(void);

int main(void)
{	
	uint32_t i;
	
	SystemInit();
	
	SerialInit();
	
	SYS->CLKEN |= (1 << SYS_CLKEN_RTCBKP_Pos);		//Backup区需要开启此时钟
	printf("RTCBKP[0]: %08X\r\n", SYS->RTCBKP[0]++);
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0);				//输出， 接LED
	
	while(1==1)
	{
		GPIO_SetBit(GPIOA, PIN5);					//点亮LED
		for(i = 0; i < 10000000; i++) __NOP();
		GPIO_ClrBit(GPIOA, PIN5);					//熄灭LED
		
		switchCLK_32KHz();
		SYS->HRCCR = (1 << SYS_HRCCR_OFF_Pos);		//关闭HRC、降低功耗
		
		SYS->SLEEP |= (1 << SYS_SLEEP_DEEP_Pos);	//进入STOP模式
		
		while(1) __NOP();	//唤醒后从头开始重新执行
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
