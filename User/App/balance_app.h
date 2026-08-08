#ifndef __BALANCE_APP_H__
#define __BALANCE_APP_H__

#include <stdint.h>

/* 1: position outer PID -> target speed -> speed middle PID -> motor angle.
 * 0: keep the tuned speed loop alone with a fixed 0 mm/s target. */
#define BALANCE_POSITION_OUTER_ENABLE       1

#if ((BALANCE_POSITION_OUTER_ENABLE != 0) && \
     (BALANCE_POSITION_OUTER_ENABLE != 1))
#error "BALANCE_POSITION_OUTER_ENABLE must be 0 or 1"
#endif

#define BALANCE_TASK_PERIOD_MS              20U

/* Position outer loop. Input is mm and output is target speed in mm/s.
 * Kp=0.75 combined with speed Kp=0.08 gives an initial cascade proportional
 * response of about 0.060 motor deg/mm. */
#define BALANCE_POSITION_KP                 1.88f			//1.88
#define BALANCE_POSITION_KI                 1.0f			//1.0
#define BALANCE_POSITION_KD                 0.4f			//0.4
#define BALANCE_POSITION_I_DEADBAND_MM      0.6f
#define BALANCE_POSITION_I_LEAK_FACTOR      0.98f
#define BALANCE_POSITION_I_LEAK_SPEED_MM_S  5.0f
#define BALANCE_POSITION_I_LIMIT_MM_S       50.0f
#define BALANCE_POSITION_SPEED_LIMIT_MM_S   100.0f

/* Speed middle loop. Input is mm/s and output is motor correction degrees.
 * These are the hardware-tuned gains; keep them while tuning the outer loop. */
#define BALANCE_SPEED_TARGET_MM_S           0.0f
#define BALANCE_SPEED_KP                    0.10f			//0.08
#define BALANCE_SPEED_KI                    0.000f
#define BALANCE_SPEED_KD                    0.0000f		//0.0004
#define BALANCE_SPEED_I_LIMIT_MM            50.0f

/* +1 preserves the installed motor direction. The control law applies the
 * coordinate-system minus sign separately. */
#define BALANCE_MOTOR_SIGN                  (+1.0f)

/* Vehicle acceleration feedforward, applied in motor-angle space. The wheel
 * command uses negative speed for forward motion, so the signed acceleration
 * and the calibrated motor sign produce the restoring rod direction. At the
 * default 30 cm/s^2 ramp this starts at 4.8 motor degrees. Tune the gain on the
 * installed linkage; set it to 0 to disable without changing the control path. */
#define BALANCE_ACCEL_FF_MOTOR_DEG_PER_M_S2 16.0f
#define BALANCE_ACCEL_FF_SIGN               (-1.0f)
#define BALANCE_ACCEL_FF_LIMIT_DEG          5.0f

#define BALANCE_TARGET_LIMIT_M              0.100f

typedef struct
{
    float target_position_mm;
    float position_mm;
    float position_error_mm;
    float target_speed_mm_s;
    float velocity_mm_s;
    float speed_error_mm_s;
    float position_integral_mm_s;
    float speed_integral_mm;
    float motor_request_deg;
    uint8_t enabled;
    uint8_t saturated;
} BalanceTelemetry_t;

void Balance_Init(void);
void Balance_Task(void);

/* Mechanical polarity only; this is not a PID tuning parameter. */
void Balance_SetMotorSign(float sign);

void Balance_SetEnable(uint8_t enable);
uint8_t Balance_IsEnabled(void);

/* Public position units remain metres to match ball_app and mission_app. */
void Balance_SetTarget(float x_m);
float Balance_GetTarget(void);
float Balance_GetError(void);
float Balance_GetCommandedMotorDeg(void);

float Balance_GetPeakError(void);
uint16_t Balance_GetSaturationTicks(void);
void Balance_ResetInstruments(void);

void Balance_PrintStatus(void);
void Balance_GetTelemetry(BalanceTelemetry_t *telemetry);

#endif
