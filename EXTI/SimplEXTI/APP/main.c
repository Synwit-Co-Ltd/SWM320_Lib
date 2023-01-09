#include "SWM320.h"

#define FAST_IRQ	0

int main(void)
{	
	SystemInit();
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0);			//输出，接LED
	
	GPIO_Init(GPIOA, PIN4, 0, 1, 0);			//输入，上拉使能，接KEY
	
	EXTI_Init(GPIOA, PIN4, EXTI_FALL_EDGE);		//下降沿触发中断
	
#if FAST_IRQ
	NVIC_EnableIRQ(GPIOA4_IRQn);
#else
	NVIC_EnableIRQ(GPIOA_IRQn);
#endif
	
	EXTI_Open(GPIOA, PIN4);
	
	while(1==1)
	{
	}
}

#if FAST_IRQ
void GPIOA4_Handler(void)
{
	EXTI_Clear(GPIOA, PIN4);
	
	GPIO_InvBit(GPIOA, PIN5);
}
#else
void GPIOA_Handler(void)
{
	if(EXTI_State(GPIOA, PIN4))
	{
		EXTI_Clear(GPIOA, PIN4);
		
		GPIO_InvBit(GPIOA, PIN5);
	}
}
#endif
