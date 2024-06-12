#ifndef __SYSTEM_SWM320_H__
#define __SYSTEM_SWM320_H__

#ifdef __cplusplus
 extern "C" {
#endif


#define PLL_OUT_DIV8	0
#define PLL_OUT_DIV4	1
#define PLL_OUT_DIV2	2


extern uint32_t SystemCoreClock;		// System Clock Frequency (Core Clock)
extern uint32_t CyclesPerUs;			// Cycles per micro second


extern void SystemInit(void);

extern void SystemCoreClockUpdate (void);


extern void switchCLK_20MHz(void);
extern void switchCLK_40MHz(void);
extern void switchCLK_32KHz(void);
extern void switchCLK_XTAL(void);
extern void switchCLK_PLL(uint32_t clksrc_xtal, uint32_t indiv, uint32_t fbdiv, uint32_t outdiv);

extern void PLLInit(uint32_t clksrc_xtal, uint32_t indiv, uint32_t fbdiv, uint32_t outdiv);


#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_SWM320_H__
