#ifndef __PENDULUM_APP_H__
#define __PENDULUM_APP_H__

#include "step_app.h"
#include <stdint.h>

/* Set to 1 to remap the four keys onto the ZDT bring-up probes. */
#define PENDULUM_PROBE_MODE          0

/* Set to 1 whenever the mechanism is attached. Destructive bare-shaft probe
 * angles are then replaced by the normal motor-angle safety limit. */
#define PENDULUM_LINKAGE_ATTACHED    1

/* Fixed-gain field tuning starts without a PC: close the ZDT loop at the
 * power-on shaft position, latch it as motor zero, then move to the measured
 * centre. The ball PID itself remains off until PC0/PC2/PC3/PC13 or PC1 starts
 * a ball-control requirement; KEY2 is reserved for Q2. */
#define PENDULUM_ENERGISE_AT_BOOT    1

/* The ball PID commands a motor-angle correction around the measured mechanical
 * centre. The centre is relative to the zero latched at boot (or by a later
 * step:1). motor:<deg> remains an absolute ZDT angle command in that coordinate
 * system. */
#define PEND_MOTOR_CENTER_DEG         39.5f
#define PEND_MOTOR_LIMIT_DEG         20.0f
#define PEND_MOTOR_MIN_DEG           (PEND_MOTOR_CENTER_DEG - PEND_MOTOR_LIMIT_DEG)
#define PEND_MOTOR_MAX_DEG           (PEND_MOTOR_CENTER_DEG + PEND_MOTOR_LIMIT_DEG)
#define PEND_MOTOR_SLEW_DEG_S        150.0f
#define PEND_TASK_PERIOD_MS          20U

/* ZDT absolute-position trajectory parameters. RPM is a speed ceiling. A
 * nonzero acceleration value keeps the driver's ramp enabled. */
#define PEND_SERVO_RPM               100U
#define PEND_SERVO_ACC               10U

/* Do not restart an absolute trajectory for an indistinguishable target. */
#define PEND_PULSE_DEADBAND          1L

void Pendulum_Init(void);
void Pendulum_Task(void);

void Pendulum_SetMotorAngle(float motor_deg);
void Pendulum_Center(void);
float Pendulum_GetTargetMotorDeg(void);
float Pendulum_GetAppliedMotorDeg(void);

/* Park the mechanism at its physical centre before calling this. The current
 * motor position becomes zero without commanding a move. */
void Pendulum_ZeroHere(void);

/* Energising re-adopts the present shaft position as zero because a released
 * mechanism may have been moved by hand. */
void Pendulum_SetEnergised(uint8_t on);
uint8_t Pendulum_IsEnergised(void);

#if PENDULUM_PROBE_MODE
void Pendulum_ProbeOverride(void);
void Pendulum_ProbeSweep(void);
void Pendulum_ProbeRepeat(void);
#endif

#endif
