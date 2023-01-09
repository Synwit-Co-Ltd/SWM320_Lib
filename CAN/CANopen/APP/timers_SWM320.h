#ifndef __TIMERS_SWM220_H__
#define __TIMERS_SWM220_H__

void setTimer(TIMEVAL value);

TIMEVAL getElapsedTime(void);

void TIM3_start(void);

void StartTimerLoop(TimerCallback_t _init_callback);

#endif //__TIMERS_SWM220_H__
