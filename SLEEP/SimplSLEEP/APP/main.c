#include "SWM320.h"


/* 注意：
 *	芯片的 ISP、SWD 引脚默认开启了上拉电阻，会增加休眠功耗，若想获得最低休眠功耗，休眠前请关闭所有引脚的上拉和下拉电阻
 */
 
 
void SerialInit(void);

int main(void)
{	
	uint32_t i;
	
	SystemInit();
	
	SerialInit();
	
	GPIO_Init(GPIOA, PIN4, 0, 1, 0);				//输入，上拉使能，接KEY
	SYS->PAWKEN |= (1 << PIN4);						//开启PA4引脚低电平唤醒
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0);				//输出，接LED
	
	while(1==1)
	{
		GPIO_SetBit(GPIOA, PIN5);					//点亮LED
		for(i = 0; i < 10000000; i++);
		GPIO_ClrBit(GPIOA, PIN5);					//熄灭LED
		
		__disable_irq();
		
		switchCLK_32KHz();
		SYS->HRCCR = (1 << SYS_HRCCR_OFF_Pos);		//关闭HRC、降低功耗
		
		SYS->SLEEP |= (1 << SYS_SLEEP_SLEEP_Pos);	//进入睡眠模式
	
		while((SYS->PAWKSR & (1 << PIN4)) == 0);	//等待唤醒条件
		SYS->PAWKSR |= (1 << PIN4);					//清除唤醒状态
		
		switchCLK_40MHz();	//切换到休眠前的时钟
		
		__enable_irq();
		
		printf("SWM320 wakeup!\r\n");
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
