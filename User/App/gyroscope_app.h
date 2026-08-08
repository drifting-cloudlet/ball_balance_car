#ifndef __GYROSCOPE_APP_H__
#define __GYROSCOPE_APP_H__

#include "main.h"
#include "gyroscope_driver.h"

void Gyroscope_Init(void);
void Gyroscope_Task(void);

/* Bias-corrected yaw rate in deg/s, and the tick it was sampled at.
 *
 * Only the rate is exported on purpose. The integrated yaw angle needs the bias
 * to be perfect or it ramps away; a rate carries the same bias as a constant
 * offset, which the boot-time stationary calibration removes outright.
 *
 * Gyroscope_Task deliberately runs on the main loop, not in the 10 ms control
 * interrupt: reading 14 bytes over 100 kHz I2C blocks for about 1.5 ms and can
 * block until timeout if the bus NACKs. That is tolerable here because this
 * signal is used directly as a damping term and never differentiated, so up to
 * one tick of staleness is pure phase lag, a few degrees at the line loop's
 * bandwidth. Gray_Task had to move into the interrupt precisely because its
 * output IS differentiated. */
extern volatile uint32_t g_yaw_rate_tick;

uint8_t Gyroscope_Rate_Is_Usable(uint32_t max_age_ms);

extern uint8_t first_gyroscope_flag;

#endif
