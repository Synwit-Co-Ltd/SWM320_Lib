#include "SWM320.h"
#include "CircleBuffer.h"

/* ˵����ֻ�е����� FIFO �������ݣ�����ָ��ʱ����δ���յ��µ�����ʱ���Żᴥ����ʱ�жϡ�
 * ��Ӧ����ϣ��ͨ�����ݼ�ʱ������Ϊ֡������ݣ������ܶԷ����͹������ٸ����ݣ�
 * ����ܲ�����ʱ�жϣ�����ͨ���ڽ��� ISR �д� RX FIFO �ж�ȡ����ʱ�����ٶ�һ��������һ���������� RX FIFO �У���ʵ�֡�
*/

volatile bool msg_rcvd = false;

CircleBuffer_t CirBuf;

void SerialInit(void);

int main(void)
{
	SystemInit();
	
	SerialInit();
   	
	while(1==1)
	{
		if(msg_rcvd)
		{
			msg_rcvd = false;
			
			UART_INTEn(UART0, UART_IT_TX_THR);
		}
	}
}


void UART0_Handler(void)
{
	uint32_t chr;
	
	if(UART_INTStat(UART0, UART_IT_RX_THR))
	{
		while((UART0->FIFO & UART_FIFO_RXLVL_Msk) > 1)
		{
			if(UART_ReadByte(UART0, &chr) == 0)
			{
				CirBuf_Write(&CirBuf, (uint8_t *)&chr, 1);
			}
		}
	}
	else if(UART_INTStat(UART0, UART_IT_RX_TOUT))
	{
		while(UART_IsRXFIFOEmpty(UART0) == 0)
		{
			if(UART_ReadByte(UART0, &chr) == 0)
			{
				CirBuf_Write(&CirBuf, (uint8_t *)&chr, 1);
			}
		}
		
		msg_rcvd = true;
	}
	
	if(UART_INTStat(UART0, UART_IT_TX_THR))
	{
		while(!UART_IsTXFIFOFull(UART0))
		{
			if(!CirBuf_Empty(&CirBuf))
			{
				CirBuf_Read(&CirBuf, (uint8_t *)&chr, 1);
				
				UART_WriteByte(UART0, chr);
			}
			else
			{
				UART_INTDis(UART0, UART_IT_TX_THR);
				
				break;
			}
		}
	}
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN2, FUNMUX0_UART0_RXD, 1);	//GPIOA.2����ΪUART0��������
	PORT_Init(PORTA, PIN3, FUNMUX1_UART0_TXD, 0);	//GPIOA.3����ΪUART0�������
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThreshold = 3;
	UART_initStruct.RXThresholdIEn = 1;
	UART_initStruct.TXThreshold = 3;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutTime = 10;		//10���ַ�ʱ����δ���յ��µ������򴥷���ʱ�ж�
	UART_initStruct.TimeoutIEn = 1;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

/****************************************************************************************************************************************** 
* ��������: fputc()
* ����˵��: printf()ʹ�ô˺������ʵ�ʵĴ��ڴ�ӡ����
* ��    ��: int ch		Ҫ��ӡ���ַ�
*			FILE *f		�ļ����
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
