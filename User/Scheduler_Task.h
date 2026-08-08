#ifndef __SCHEDULER_TASK_H__
#define __SCHEDULER_TASK_H__

#include "MyDefine.h"

void System_Init(void);

/* Measured duration of the 10 ms control tick, in microseconds. gray_* covers
 * just the 64 ADC conversions of the grayscale frame; ctrl_* covers the whole
 * chain (grayscale, encoder, PID, data log). */
void System_TickStats(uint32_t *gray_us, uint32_t *gray_us_max,
                      uint32_t *ctrl_us, uint32_t *ctrl_us_max);
void System_TickStatsReset(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
