#include "SWM320.h"


int main(void)
{	
	SystemInit();
	
	GPIO_Init(GPIOA, PIN4, 0, 1, 0);			//���룬����ʹ�ܣ���KEY
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0);			//����� ��LED
	
	while(1==1)
	{
		if(GPIO_GetBit(GPIOA, PIN4) == 0)		//����������
		{
			GPIO_AtomicSetBit(GPIOA, PIN5);
		}
		else
		{
			GPIO_AtomicClrBit(GPIOA, PIN5);
		}
	}
}
