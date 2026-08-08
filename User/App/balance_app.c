#include "balance_app.h"
#include "MyDefine.h"
#include <math.h>

#define BALANCE_DT_S       ((float)BALANCE_TASK_PERIOD_MS / 1000.0f)
#define BALANCE_M_TO_MM    1000.0f

static uint8_t balance_enabled = 0U;

/* Mission_Task writes the target in the 10 ms timer ISR. Balance_Task reads it
 * on the main loop; a float store is atomic on Cortex-M4. */
static volatile float balance_target_m = 0.0f;

static float balance_error_m = 0.0f;
static float balance_velocity_mm_s = 0.0f;
static float balance_target_speed_mm_s = BALANCE_SPEED_TARGET_MM_S;

#if BALANCE_POSITION_OUTER_ENABLE
static float balance_position_integral_mm_s = 0.0f;
static float balance_position_derivative_mm_s = 0.0f;
static const float balance_position_kp = BALANCE_POSITION_KP;
static const float balance_position_ki = BALANCE_POSITION_KI;
static const float balance_position_kd = BALANCE_POSITION_KD;
#endif

static float balance_speed_error_mm_s = 0.0f;
static float balance_speed_integral_mm = 0.0f;
static float balance_speed_derivative_mm_s2 = 0.0f;
static float balance_speed_prev_error_mm_s = 0.0f;
static uint8_t balance_speed_primed = 0U;
static const float balance_speed_kp = BALANCE_SPEED_KP;
static const float balance_speed_ki = BALANCE_SPEED_KI;
static const float balance_speed_kd = BALANCE_SPEED_KD;

static float balance_motor_deg = PEND_MOTOR_CENTER_DEG;
static float balance_accel_ff_deg = 0.0f;
static float balance_peak_err_m = 0.0f;
static uint16_t balance_sat_ticks = 0U;
static uint8_t balance_saturated = 0U;
static float balance_motor_sign = BALANCE_MOTOR_SIGN;

static BallState_t balance_last_state = BALL_STATE_LOST;
static uint8_t balance_state_seen = 0U;

static float Balance_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static const char *Balance_StateName(BallState_t state)
{
    switch (state)
    {
        case BALL_STATE_LOST:     return "LOST";
        case BALL_STATE_COASTING: return "COASTING";
        case BALL_STATE_TRACKING: return "TRACKING";
        default:                  return "ERROR";
    }
}

static void Balance_ResetControllerState(void)
{
    balance_error_m = 0.0f;
    balance_velocity_mm_s = 0.0f;
    balance_target_speed_mm_s = BALANCE_SPEED_TARGET_MM_S;
#if BALANCE_POSITION_OUTER_ENABLE
    balance_position_integral_mm_s = 0.0f;
    balance_position_derivative_mm_s = 0.0f;
#endif
    balance_speed_error_mm_s = 0.0f;
    balance_speed_integral_mm = 0.0f;
    balance_speed_derivative_mm_s2 = 0.0f;
    balance_speed_prev_error_mm_s = 0.0f;
    balance_speed_primed = 0U;
    balance_accel_ff_deg = 0.0f;
}

void Balance_Init(void)
{
    balance_enabled = 0U;
    balance_target_m = 0.0f;
    Balance_ResetControllerState();
    balance_motor_deg = PEND_MOTOR_CENTER_DEG;
    balance_motor_sign = BALANCE_MOTOR_SIGN;
    balance_state_seen = 0U;
    Balance_ResetInstruments();
    Pendulum_Center();
#if BALANCE_POSITION_OUTER_ENABLE
    Uart_Printf(DEBUG_UART,
                "Balance_Init CASCADE outer=%.3f/%.3f/%.3f "
                "middle=%.4f/%.4f/%.4f sign=%+.0f\r\n",
                balance_position_kp, balance_position_ki,
                balance_position_kd, balance_speed_kp,
                balance_speed_ki, balance_speed_kd, balance_motor_sign);
#else
    Uart_Printf(DEBUG_UART,
                "Balance_Init SPEED-ONLY kp=%.4f ki=%.4f kd=%.4f "
                "target=%.1fmm/s sign=%+.0f\r\n",
                balance_speed_kp, balance_speed_ki, balance_speed_kd,
                BALANCE_SPEED_TARGET_MM_S, balance_motor_sign);
#endif
}

void Balance_SetEnable(uint8_t enable)
{
    Balance_ResetControllerState();
    balance_state_seen = 0U;
    Balance_ResetInstruments();

    if (enable == 0U)
    {
        balance_enabled = 0U;
        balance_motor_deg = PEND_MOTOR_CENTER_DEG;
        Pendulum_Center();
        Uart_Printf(DEBUG_UART, "balance off\r\n");
        return;
    }

    balance_enabled = 1U;
#if BALANCE_POSITION_OUTER_ENABLE
    Uart_Printf(DEBUG_UART, "balance CASCADE on target=%.1fmm\r\n",
                balance_target_m * BALANCE_M_TO_MM);
#else
    Uart_Printf(DEBUG_UART,
                "balance SPEED-ONLY on target=%.1fmm/s; position ignored\r\n",
                BALANCE_SPEED_TARGET_MM_S);
#endif
}

uint8_t Balance_IsEnabled(void)
{
    return balance_enabled;
}

void Balance_SetTarget(float x_m)
{
    balance_target_m = Balance_Clamp(x_m, BALANCE_TARGET_LIMIT_M);
}

float Balance_GetTarget(void)
{
    return balance_target_m;
}

float Balance_GetError(void)
{
    return balance_error_m;
}

float Balance_GetCommandedMotorDeg(void)
{
    return balance_motor_deg;
}

float Balance_GetPeakError(void)
{
    return balance_peak_err_m;
}

uint16_t Balance_GetSaturationTicks(void)
{
    return balance_sat_ticks;
}

void Balance_ResetInstruments(void)
{
    balance_peak_err_m = 0.0f;
    balance_sat_ticks = 0U;
}

void Balance_SetMotorSign(float sign)
{
    balance_motor_sign = (sign < 0.0f) ? -1.0f : +1.0f;
    Balance_ResetControllerState();
}

void Balance_Task(void)
{
#if BALANCE_POSITION_OUTER_ENABLE
    float error_mm;
#endif
    float pid_output_deg;
    float motor_correction_deg;
    BallState_t state;

    balance_saturated = 0U;

    if (balance_enabled == 0U)
    {
        return;
    }

    state = Ball_GetState();
    if ((balance_state_seen == 0U) || (state != balance_last_state))
    {
        Uart_Printf(DEBUG_UART, "balance: ball %s%s\r\n",
                    Balance_StateName(state),
                    (state == BALL_STATE_LOST)
                        ? " - motor held at center, run ball" : "");
        balance_last_state = state;
        balance_state_seen = 1U;
    }

    if (state == BALL_STATE_LOST)
    {
        Balance_ResetControllerState();
        balance_motor_deg = PEND_MOTOR_CENTER_DEG;
        Pendulum_Center();
        return;
    }

    balance_error_m = balance_target_m - Ball_GetPosition();
#if BALANCE_POSITION_OUTER_ENABLE
    error_mm = balance_error_m * BALANCE_M_TO_MM;
#endif
    balance_velocity_mm_s = Ball_GetVelocity() * BALANCE_M_TO_MM;

    if (fabsf(balance_error_m) > balance_peak_err_m)
    {
        balance_peak_err_m = fabsf(balance_error_m);
    }

#if BALANCE_POSITION_OUTER_ENABLE
    /* Derivative on measurement avoids a target-change kick. */
    balance_position_derivative_mm_s = -balance_velocity_mm_s;

    if ((state == BALL_STATE_TRACKING) &&
        (fabsf(error_mm) > BALANCE_POSITION_I_DEADBAND_MM))
    {
        balance_position_integral_mm_s += error_mm * BALANCE_DT_S;
        balance_position_integral_mm_s = Balance_Clamp(
            balance_position_integral_mm_s,
            BALANCE_POSITION_I_LIMIT_MM_S);
    }
    else if ((state == BALL_STATE_TRACKING) &&
             (fabsf(balance_velocity_mm_s) <=
              BALANCE_POSITION_I_LEAK_SPEED_MM_S))
    {
        balance_position_integral_mm_s *=
            BALANCE_POSITION_I_LEAK_FACTOR;
        if (fabsf(balance_position_integral_mm_s) < 0.001f)
        {
            balance_position_integral_mm_s = 0.0f;
        }
    }

    balance_target_speed_mm_s =
        (balance_position_kp * error_mm) +
        (balance_position_ki * balance_position_integral_mm_s) +
        (balance_position_kd * balance_position_derivative_mm_s);
    balance_target_speed_mm_s = Balance_Clamp(
        balance_target_speed_mm_s,
        BALANCE_POSITION_SPEED_LIMIT_MM_S);
#else
    balance_target_speed_mm_s = BALANCE_SPEED_TARGET_MM_S;
#endif

    balance_speed_error_mm_s =
        balance_target_speed_mm_s - balance_velocity_mm_s;

    if (balance_speed_primed == 0U)
    {
        balance_speed_prev_error_mm_s = balance_speed_error_mm_s;
        balance_speed_derivative_mm_s2 = 0.0f;
        balance_speed_primed = 1U;
    }
    else
    {
        balance_speed_derivative_mm_s2 =
            (balance_speed_error_mm_s - balance_speed_prev_error_mm_s) /
            BALANCE_DT_S;
        balance_speed_prev_error_mm_s = balance_speed_error_mm_s;
    }

    /* Freeze both integrators while coasting on predicted ball state. */
    if (state == BALL_STATE_TRACKING)
    {
        balance_speed_integral_mm += balance_speed_error_mm_s * BALANCE_DT_S;
        balance_speed_integral_mm = Balance_Clamp(
            balance_speed_integral_mm, BALANCE_SPEED_I_LIMIT_MM);
    }

    pid_output_deg =
        (balance_speed_kp * balance_speed_error_mm_s) +
        (balance_speed_ki * balance_speed_integral_mm) +
        (balance_speed_kd * balance_speed_derivative_mm_s2);

    /* With +x toward the car front, a positive speed error needs the actuator
     * to move in the restoring direction. */
    motor_correction_deg = -balance_motor_sign * pid_output_deg;

    /* Feed the known chassis acceleration directly to the rod. This acts before
     * ball displacement feedback and therefore prevents most of the startup
     * excursion instead of correcting it afterwards. */
    balance_accel_ff_deg =
        balance_motor_sign * BALANCE_ACCEL_FF_SIGN *
        (PID_GetCommandAccelerationCmS2() * 0.01f) *
        BALANCE_ACCEL_FF_MOTOR_DEG_PER_M_S2;
    balance_accel_ff_deg = Balance_Clamp(balance_accel_ff_deg,
                                         BALANCE_ACCEL_FF_LIMIT_DEG);
    motor_correction_deg += balance_accel_ff_deg;

    if (fabsf(motor_correction_deg) > PEND_MOTOR_LIMIT_DEG)
    {
        balance_saturated = 1U;
        if (balance_sat_ticks < 0xFFFFU)
        {
            ++balance_sat_ticks;
        }
    }

    motor_correction_deg = Balance_Clamp(motor_correction_deg,
                                         PEND_MOTOR_LIMIT_DEG);
    balance_motor_deg = PEND_MOTOR_CENTER_DEG + motor_correction_deg;
    Pendulum_SetMotorAngle(balance_motor_deg);
}

void Balance_PrintStatus(void)
{
#if BALANCE_POSITION_OUTER_ENABLE
    Uart_Printf(DEBUG_UART,
                "bal CASCADE %s x_tgt=%.1f x_err=%.1fmm v_tgt=%.1f "
                "v=%.1fmm/s motor=%.2fdeg\r\n",
                (balance_enabled != 0U) ? "ON" : "OFF",
                balance_target_m * BALANCE_M_TO_MM,
                balance_error_m * BALANCE_M_TO_MM,
                balance_target_speed_mm_s, balance_velocity_mm_s,
                balance_motor_deg);
    Uart_Printf(DEBUG_UART,
                "outer kp=%.3f ki=%.3f kd=%.3f i=%.2f d=%.1f\r\n",
                balance_position_kp, balance_position_ki,
                balance_position_kd, balance_position_integral_mm_s,
                balance_position_derivative_mm_s);
#else
    Uart_Printf(DEBUG_UART,
                "bal SPEED %s v_tgt=%.1f v=%.1f v_err=%.1fmm/s "
                "motor=%.2fdeg\r\n",
                (balance_enabled != 0U) ? "ON" : "OFF",
                balance_target_speed_mm_s, balance_velocity_mm_s,
                balance_speed_error_mm_s, balance_motor_deg);
#endif
    Uart_Printf(DEBUG_UART,
                "middle kp=%.4f ki=%.4f kd=%.4f i=%.2f d=%.1f ff=%.2f "
                "peak=%.1fmm sat=%u sign=%+.0f\r\n",
                balance_speed_kp, balance_speed_ki, balance_speed_kd,
                balance_speed_integral_mm,
                balance_speed_derivative_mm_s2,
                balance_accel_ff_deg,
                balance_peak_err_m * BALANCE_M_TO_MM,
                (unsigned int)balance_sat_ticks,
                balance_motor_sign);
}

void Balance_GetTelemetry(BalanceTelemetry_t *telemetry)
{
    float position_mm;

    if (telemetry == NULL)
    {
        return;
    }

    telemetry->target_position_mm = balance_target_m * BALANCE_M_TO_MM;
    telemetry->target_speed_mm_s = balance_target_speed_mm_s;

    if (balance_enabled != 0U)
    {
        telemetry->position_error_mm = balance_error_m * BALANCE_M_TO_MM;
        telemetry->position_mm =
            telemetry->target_position_mm - telemetry->position_error_mm;
        telemetry->velocity_mm_s = balance_velocity_mm_s;
        telemetry->speed_error_mm_s = balance_speed_error_mm_s;
    }
    else
    {
        position_mm = Ball_GetPosition() * BALANCE_M_TO_MM;
        telemetry->position_mm = position_mm;
        telemetry->position_error_mm =
            telemetry->target_position_mm - position_mm;
        telemetry->velocity_mm_s = Ball_GetVelocity() * BALANCE_M_TO_MM;
        telemetry->speed_error_mm_s =
            telemetry->target_speed_mm_s - telemetry->velocity_mm_s;
    }
#if BALANCE_POSITION_OUTER_ENABLE
    telemetry->position_integral_mm_s = balance_position_integral_mm_s;
#else
    telemetry->position_integral_mm_s = 0.0f;
#endif
    telemetry->speed_integral_mm = balance_speed_integral_mm;
    telemetry->motor_request_deg = balance_motor_deg;
    telemetry->enabled = balance_enabled;
    telemetry->saturated = balance_saturated;
}
