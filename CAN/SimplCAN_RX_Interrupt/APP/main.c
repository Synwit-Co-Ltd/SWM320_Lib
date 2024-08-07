#include "SWM320.h"


/* ��������ʾ����˵����
	һ���� CAN �����շ������շ������������� CAN �ڵ㣬ִ�г���ͨ���������ֻᷢ�� CAN->TXERR ����𽥵����� 128�����˺��ٵ���������ԭ��ο� CAN spec ������˵����
	
		3. When a TRANSMITTER sends an ERROR FLAG the TRANSMIT ERROR COUNT is increased by 8.

		Exception 1:
		If the TRANSMITTER is 'error passive' and detects an ACKNOWLEDGMENT ERROR because of not detecting
		a 'dominant' ACK and does not detect a 'dominant' bit while sending its PASSIVE ERROR FLAG.
	
	������ʱ ACK ERROR ���ٵ��� CAN->TXERR���� CAN ���շ����Ͽ������� BIT ERROR��ͨ���������ֿɷ��� CAN->TXERR ���������� 256��
		���� CAN->SR.BUSOFF �� CAN->CR.RST ��λ��Ȼ�� CAN->TXERR ֵ��� 127����ʱ CAN �ڵ㴦�� Bus Off ״̬��CAN ���������ڸ�λ״̬��
	
	����CAN->CR.RST ���������󣬽� CAN ���շ���������CAN �ڵ��˳� Bus Off ״̬
*/


void SerialInit(void);

uint32_t sameBits(uint32_t std_id[], uint32_t std_n, uint32_t ext_id[], uint32_t ext_n);

int main(void)
{
	uint32_t i;
	CAN_InitStructure CAN_initStruct;
	uint8_t tx_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	
	SystemInit();
	
	SerialInit();
	
	PORT_Init(PORTA, PIN4, FUNMUX0_CAN_RX, 1);	//GPIOA.4����ΪCAN��������
	PORT_Init(PORTA, PIN5, FUNMUX1_CAN_TX, 0);	//GPIOA.5����ΪCAN�������
	
	CAN_initStruct.Mode = CAN_MODE_NORMAL;
	CAN_initStruct.CAN_BS1 = CAN_BS1_5tq;
	CAN_initStruct.CAN_BS2 = CAN_BS2_4tq;
	CAN_initStruct.CAN_SJW = CAN_SJW_2tq;
	CAN_initStruct.Baudrate = 100000;
	CAN_initStruct.FilterMode = CAN_FILTER_32b;
   	switch(0)
	{
	case 0:		// ����IDΪ����ֵ��֡
		CAN_initStruct.FilterMask32b  = 0xFFFFFFFF;				// 0 must match    1 don't care
		CAN_initStruct.FilterCheck32b = 0xFFFFFFFF;
		break;
	
	case 1:		// ����IDΪ0x122�ı�׼֡
		CAN_initStruct.FilterMask32b = ~(0x7FFu << 21);
		CAN_initStruct.FilterCheck32b = (0x122  << 21);			// ��׼֡ID�ڵ�21λ��31λ������ֲ�
		break;
	
	case 2:		// ����IDΪ0x122����չ֡
		CAN_initStruct.FilterMask32b = ~(0x1FFFFFFFu << 3);
		CAN_initStruct.FilterCheck32b = (0x00000122  << 3);		// ��չ֡ID�ڵ� 3λ��31λ������ֲ�
		break;
	
	case 3:		// ����IDΪ0x12X�ı�׼֡��X��ʾID�ĵ�4λ��ʲôֵ����ν
		CAN_initStruct.FilterMask32b = ~(0x7F0u << 21);
		CAN_initStruct.FilterCheck32b = (0x122  << 21);
		break;
	
	case 4:		// ����IDΪ0x122��0x101�ı�׼֡
		CAN_initStruct.FilterMode = CAN_FILTER_16b;
		CAN_initStruct.FilterMask16b1 = (uint16_t)~(0x7FFu << 5);
		CAN_initStruct.FilterCheck16b1 = (0x122  << 5);
		CAN_initStruct.FilterMask16b2 = (uint16_t)~(0x7FFu << 5);
		CAN_initStruct.FilterCheck16b2 = (0x101  << 5);
		break;
	
	case 5:		// ������IDΪ0x122��0x235��0x450�ı�׼֡��Ҳ����IDΪ0x101, 0x235, 0x1780����չ֡
		{		// ������ֻ����IDΪ��6��ֵ��֡�����ǽ��վ������ٵ�֡�����ܱ�֤IDΪҪ��ֵ��֡���ܽ��յ�
		uint32_t stdID[] = {0x122, 0x235, 0x450};
		uint32_t extID[] = {0x101, 0x235, 0x1780};
		
		CAN_initStruct.FilterMask32b = ~(sameBits(stdID, 3, extID, 3) << 3);
		CAN_initStruct.FilterCheck32b = (extID[0]  << 3);
		// ���ߣ�
// 		CAN_initStruct.FilterCheck32b = (stdID[0]  << 21);
		}
		break;
	}
	CAN_initStruct.RXNotEmptyIEn = 1;
	CAN_initStruct.RXOverflowIEn = 0;
	CAN_initStruct.ArbitrLostIEn = 0;
	CAN_initStruct.ErrPassiveIEn = 1;
	CAN_Init(CAN, &CAN_initStruct);
	
	CAN_INTEn(CAN, CAN_IT_ERR_WARN | CAN_IT_RX_OVERFLOW);
	
	CAN_Open(CAN);
	
	while(1==1)
	{
		/* ע�⣺CAN->CR.RST == 1 ʱִ�� CAN_Transmit() ���޸� Filter ���ã������޷���������� CAN ֡ */
		while((CAN->CR & CAN_CR_RST_Msk) == 0)
		{
			CAN_Transmit(CAN, CAN_FRAME_STD, 0x60D, tx_data, 8, 1);
			
			while(CAN_TXComplete(CAN) == 0) __NOP();
			
			printf("\r\nCAN->TXERR: %d\r\n", CAN->TXERR);
			
			for(i = 0; i < SystemCoreClock/16; i++) __NOP();
		}
	}
}

void CAN_Handler(void)
{
	uint32_t i;
	uint32_t int_sr = CAN_INTStat(CAN);
	
	if(int_sr & CAN_IF_RXDA_Msk)
	{
		CAN_RXMessage msg;
		
		CAN_Receive(CAN, &msg);
		
		if(msg.size > 0)
		{
			printf("\r\nReceive %s: %08X, ", msg.format == CAN_FRAME_STD ? "STD" : "EXT", msg.id);
			for(i = 0; i < msg.size; i++) printf("%02X, ", msg.data[i]);
		}
		else if(msg.remote == 1)	//Զ��֡
		{
			printf("\r\nReceive %s Remote Request", msg.format == CAN_FRAME_STD ? "STD" : "EXT");
		}
	}
	
	if(int_sr & CAN_IF_RXOV_Msk)
	{
		printf("\r\nCAN RX Overflow\r\n");
		
		CAN_Close(CAN);
		for(i = 0; i < CyclesPerUs; i++) __NOP();
		CAN_Open(CAN);
	}
	
	if(int_sr & CAN_IF_ERRWARN_Msk)
	{
		if(CAN->SR & CAN_SR_BUSOFF_Msk)
		{
			printf("\r\nCAN Bus Off\r\n");
			printf("\r\nCAN->CR.RST = %d\r\n", CAN->CR & CAN_CR_RST_Msk ? 1 : 0);	// ���� BusOff ʱ CR.RST �Զ���λ�����븴λģʽ
			
			CAN_Open(CAN);			// ��� CR.RST λ���˳���λģʽ��������������ģʽ
		}
		else if(CAN->SR & CAN_SR_ERRWARN_Msk)
		{
			printf("\r\nCAN Error Warning\r\n");
		}
	}
	
	if(int_sr & CAN_IF_ERRPASS_Msk)
	{
		printf("\r\nCAN Error Passive\r\n");
	}
}

/****************************************************************************************************************************************** 
* ��������: sameBits()
* ����˵��: �ҵ�����ID����ͬλ��ֵ��ͬ��λ����������ID�ĵ�10λ����0����1���򷵻�ֵ�ĵ�10λΪ1
* ��    ��: 
* ��    ��: 
* ע������: ��
******************************************************************************************************************************************/
uint32_t sameBits(uint32_t std_id[], uint32_t std_n, uint32_t ext_id[], uint32_t ext_n)
{
	uint32_t i, j;

	uint32_t mask = 0;
	
	for(i = 0; i < std_n; i++)
		std_id[i] = std_id[i] << 18;
	
	for(i = 0; i < std_n-1; i++)
	{
		for(j = i+1; j < std_n; j++)
		{
			mask |= std_id[i] ^ std_id[j];
		}
	}
	
	for(i = 0; i < ext_n-1; i++)
	{
		for(j = i+1; j < ext_n; j++)
		{
			mask |= ext_id[i] ^ ext_id[j];
		}
	}
	
	return ~mask;
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
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutIEn = 0;
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
