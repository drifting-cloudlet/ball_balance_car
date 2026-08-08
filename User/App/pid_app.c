#include "pid_app.h"

#define SPEED_TARGET_MIN  (-120)
#define SPEED_TARGET_MAX  (0)
#define LINE_DATA_MAX_AGE_MS  50U

static int PID_Clamp_Speed_Target(int speed)
{
    if (speed > SPEED_TARGET_MAX)
    {
        return SPEED_TARGET_MAX;
    }

    if (speed < SPEED_TARGET_MIN)
    {
        return SPEED_TARGET_MIN;
    }

    return speed;
}

/* Runtime tunable so the dead zone can be found without a reflash. */
volatile float speed_ff_duty = SPEED_FF_DUTY_PERCENT;
volatile float line_gyro_kd = LINE_GYRO_KD;

volatile int basic_speed = -50;
volatile int left_speed_target = 0;
volatile int right_speed_target = 0;
volatile float line_speed_correction = 0.0f;

volatile float speed_cmd_cm_s = 0.0f;
static volatile float speed_cmd_accel_cm_s2 = 0.0f;
static volatile uint8_t line_hold_active = 0U;
static float speed_feedback_left_cm_s = 0.0f;
static float speed_feedback_right_cm_s = 0.0f;

/* Slew the common-mode target toward basic_speed once per control tick. */
static void PID_RampCommonMode(void)
{
    float previous = speed_cmd_cm_s;
    float target = (float)basic_speed;

    if (target > (speed_cmd_cm_s + SPEED_RAMP_PER_TICK))
    {
        speed_cmd_cm_s += SPEED_RAMP_PER_TICK;
    }
    else if (target < (speed_cmd_cm_s - SPEED_RAMP_PER_TICK))
    {
        speed_cmd_cm_s -= SPEED_RAMP_PER_TICK;
    }
    else
    {
        speed_cmd_cm_s = target;
    }

    speed_cmd_accel_cm_s2 =
        (speed_cmd_cm_s - previous) / SPEED_CONTROL_DT_S;

}

float PID_GetCommandAccelerationCmS2(void)
{
    return speed_cmd_accel_cm_s2;
}

PID_T pid_speed_left;
PID_T pid_speed_right;
PID_T pid_line;

#if !SPEED_PID_DEBUG_MODE
static uint8_t line_has_lock = 0U;
static uint8_t line_derivative_needs_prime = 1U;
#endif

/* Wheel speed loops: positional PD in duty percent, matching the M0 reference.
 *
 * The previous incremental form multiplied kp by the CHANGE in error. With
 * kp = 35 and an encoder that quantises to +-1.4 cm/s per 10 ms sample, that
 * alone injected +-98 counts of output jitter per tick, accumulating into out
 * as a random walk. Worse, a 13 cm/s swing in error moved the output by 473
 * counts in one tick, which combined with the driver dead band to slam the
 * command from -550 to +550: full reverse on a motor still driving forward.
 *
 * Positional PD with kd = 0.81 turns the same quantisation into +-1.1% of duty.
 * ki stays zero: the dead-zone feedforward already supplies the steady bias
 * that an integrator would otherwise have to build. */
static const PidParams_t pid_params_left = {
    .kp = 6.5f,
    .ki = 0.0f,
    .kd = 0.81f,
    .out_min = -100.0f,
    .out_max = 100.0f,
};

static const PidParams_t pid_params_right = {
    .kp = 6.5f,
    .ki = 0.0f,
    .kd = 0.81f,
    .out_min = -100.0f,
    .out_max = 100.0f,
};

static const PidParams_t pid_params_line = {
    .kp = 0.03f,
    .ki = 0.0f,
    .kd = 0.05f,
    .out_min = -15.0f,
    .out_max = 15.0f,
};

static int PID_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

static void PID_ResetAll(void)
{
    pid_reset(&pid_line);
    pid_reset(&pid_speed_left);
    pid_reset(&pid_speed_right);
    line_speed_correction = 0.0f;
    line_hold_active = 0U;
    speed_feedback_left_cm_s = 0.0f;
    speed_feedback_right_cm_s = 0.0f;
#if !SPEED_PID_DEBUG_MODE
    line_has_lock = 0U;
    line_derivative_needs_prime = 1U;
#endif
}

static void PID_UpdateSpeedFeedback(void)
{
    speed_feedback_left_cm_s += SPEED_FEEDBACK_LPF_ALPHA *
        (left_encoder.speed_cm_s - speed_feedback_left_cm_s);
    speed_feedback_right_cm_s += SPEED_FEEDBACK_LPF_ALPHA *
        (right_encoder.speed_cm_s - speed_feedback_right_cm_s);
}

#if !SPEED_PID_DEBUG_MODE
static void PID_MixLineTargets(float correction)
{
    float authority = fabsf(speed_cmd_cm_s) * LINE_RAMP_STEERING_FRACTION;

    /* Reserve common-mode motion for both wheels at both ends of the ramp.
     * Without this, one wheel is clamped to zero during launch and Q4 braking
     * can end in a pivot. At normal cruise the line loop's own +/-10 cm/s limit
     * is smaller, so this does not reduce steering authority. */
    correction = pid_constrain(correction, -authority, authority);

    line_speed_correction = correction;
    left_speed_target = PID_Clamp_Speed_Target(
        PID_RoundToInt(speed_cmd_cm_s + correction));
    right_speed_target = PID_Clamp_Speed_Target(
        PID_RoundToInt(speed_cmd_cm_s - correction));
}

static void PID_ApplyLineCorrection(void)
{
    float correction = pid_calculate_positional(&pid_line,
                                                 g_line_position_error);

    /* Gyro damping, added before the limit so it cannot push the command past
     * the differential authority. Skipped outright when the rate is stale or the
     * bias has not converged: a damping term fed garbage is worse than none. */
    if ((line_gyro_kd != 0.0f) &&
        (Gyroscope_Rate_Is_Usable(LINE_GYRO_MAX_AGE_MS) != 0U))
    {
        correction += LINE_GYRO_SIGN * line_gyro_kd * Gyroscope_Get_Yaw_Rate();
    }

    correction = pid_constrain(correction,
                               pid_params_line.out_min,
                               pid_params_line.out_max);
    /* Forward is negative: a negative correction makes the left wheel faster.
     * Built on the ramped command, not on basic_speed, so the start transient is
     * bounded and its acceleration is the one published to the balance loop. */
    PID_MixLineTargets(correction);
}

/* After the first valid line lock, every line-following mode rides through a
 * dropout by preserving the exact last wheel targets. Q4 braking is the one
 * exception: keep the last steering correction but let common speed reach
 * zero. Reacquisition primes the derivative before normal updates resume. */
static uint8_t PID_HoldLastTargetsForLineLoss(void)
{
    MissionMode_t mode = Mission_GetMode();
    MissionState_t state = Mission_GetState();
    uint8_t q4_braking = ((mode == MISSION_MODE_Q4) &&
                          (state == MISSION_BRAKE)) ? 1U : 0U;

    if (line_has_lock == 0U)
    {
        return 0U;
    }

    line_derivative_needs_prime = 1U;

    if (q4_braking != 0U)
    {
        PID_MixLineTargets(line_speed_correction);
    }
    else
    {
        speed_cmd_cm_s = ((float)left_speed_target +
                          (float)right_speed_target) * 0.5f;
        speed_cmd_accel_cm_s2 = 0.0f;
    }

    line_hold_active = 1U;
    return 1U;
}

static uint8_t PID_UpdateLineTargets(void)
{
    if (Gray_Is_Data_Fresh(LINE_DATA_MAX_AGE_MS) == 0U)
    {
        if (PID_HoldLastTargetsForLineLoss() != 0U)
        {
            return 1U;
        }

        pid_reset(&pid_line);
        line_speed_correction = 0.0f;
        line_hold_active = 0U;
        line_has_lock = 0U;
        line_derivative_needs_prime = 1U;
        return 0U;
    }

    if (g_line_detected != 0U)
    {
        line_has_lock = 1U;
        line_hold_active = 0U;
        if (line_derivative_needs_prime != 0U)
        {
            /* A dropout can leave last_error hundreds of sensor units behind
             * the reacquired line. Prime the history so the first valid frame
             * uses P only instead of producing a full-scale derivative kick. */
            pid_line.last_error = pid_line.target - g_line_position_error;
            line_derivative_needs_prime = 0U;
        }
        PID_ApplyLineCorrection();
        return 1U;
    }

    if (line_has_lock == 0U)
    {
        line_hold_active = 0U;
        line_derivative_needs_prime = 1U;
        return 0U;
    }

    if (PID_HoldLastTargetsForLineLoss() != 0U)
    {
        return 1U;
    }
    return 0U;
}
#endif

void PID_Init(void)
{
    basic_speed = PID_Clamp_Speed_Target(basic_speed);
    left_speed_target = 0;
    right_speed_target = 0;

    pid_init(&pid_speed_left,
             pid_params_left.kp, pid_params_left.ki, pid_params_left.kd,
             0.0f, pid_params_left.out_max);

    pid_init(&pid_speed_right,
             pid_params_right.kp, pid_params_right.ki, pid_params_right.kd,
             0.0f, pid_params_right.out_max);

    pid_init(&pid_line,
             pid_params_line.kp, pid_params_line.ki, pid_params_line.kd,
             0.0f, pid_params_line.out_max);

    pid_set_target(&pid_speed_left, 0.0f);
    pid_set_target(&pid_speed_right, 0.0f);
    pid_set_target(&pid_line, 0.0f);
}

volatile uint8_t pid_running = 0U;

/* Counted in the 10 ms control interrupt so the main loop can tell a line that
 * keeps dropping out from a speed loop that cannot settle. Both look identical
 * from the outside: the wheels judder. */
volatile uint32_t pid_drive_ticks = 0U;
volatile uint32_t pid_brake_ticks = 0U;

/* Speed-loop monitor.
 *
 * "Is the speed smooth" is a question about the shape of the signal between
 * prints, so sampling it once per print cannot answer it: at 10 Hz over a 9600
 * baud link, a 3 Hz ripple aliases into a slow drift and looks fine. These
 * min/max/sum accumulators run in the 10 ms control interrupt and are drained
 * by the monitor task, so one printed line carries the full envelope of the
 * interval it covers.
 *
 * zero_clamp counts ticks where the output would have opposed the target and
 * was forced to zero; saturated counts ticks pinned at +-100% duty. A speed
 * loop that is bang-banging shows up as a large zero_clamp count long before
 * the averages look wrong. */
static volatile float mon_left_min = 0.0f;
static volatile float mon_left_max = 0.0f;
static volatile float mon_left_sum = 0.0f;
static volatile float mon_right_min = 0.0f;
static volatile float mon_right_max = 0.0f;
static volatile float mon_right_sum = 0.0f;
static volatile uint16_t mon_samples = 0U;
static volatile uint16_t mon_zero_clamp = 0U;
static volatile uint16_t mon_saturated = 0U;

volatile uint32_t pid_monitor_period_ms = 0U;
static uint32_t pid_monitor_last_tick = 0U;

static void PID_MonitorAccumulate(float left, float right)
{
    if (mon_samples == 0U)
    {
        mon_left_min = left;
        mon_left_max = left;
        mon_right_min = right;	
        mon_right_max = right;
        mon_left_sum = 0.0f;
        mon_right_sum = 0.0f;
    }
    else
    {
        if (left < mon_left_min)   { mon_left_min = left; }
        if (left > mon_left_max)   { mon_left_max = left; }
        if (right < mon_right_min) { mon_right_min = right; }
        if (right > mon_right_max) { mon_right_max = right; }
    }

    mon_left_sum += left;
    mon_right_sum += right;
    ++mon_samples;
}

void PID_ResetCounters(void)
{
    pid_drive_ticks = 0U;
    pid_brake_ticks = 0U;
}

void PID_PrintStatus(void)
{
    uint32_t drive = pid_drive_ticks;
    uint32_t brake = pid_brake_ticks;
    uint32_t total = drive + brake;

    Uart_Printf(DEBUG_UART,
                "pid %s base=%d tgt=%d,%d cur=%.1f,%.1f duty=%.1f,%.1f%% ff=%.1f%%\r\n",
                (pid_running != 0U) ? "ON" : "OFF",
                basic_speed,
                left_speed_target, right_speed_target,
                left_encoder.speed_cm_s, right_encoder.speed_cm_s,
                pid_speed_left.out, pid_speed_right.out,
                speed_ff_duty);
    Uart_Printf(DEBUG_UART,
                "pid line=%u fresh=%u hold=%u err=%.1f corr=%.1f | drive=%u brake=%u (%u%% braked)\r\n",
                g_line_detected,
                Gray_Is_Data_Fresh(LINE_DATA_MAX_AGE_MS),
                line_hold_active,
                g_line_position_error,
                line_speed_correction,
                drive, brake,
                (total != 0U) ? (unsigned int)((brake * 100U) / total) : 0U);
}

void PID_Set_Enable(uint8_t enable)
{
    pid_running = 0U;
    PID_ResetCounters();
    PID_ResetAll();
    left_speed_target = 0;
    right_speed_target = 0;
    pid_set_target(&pid_speed_left, 0.0f);
    pid_set_target(&pid_speed_right, 0.0f);

    /* The ramp always restarts from a standstill. Carrying the old command over
     * would let a re-enable step straight back to cruise, which is the exact
     * transient the ramp exists to remove. */
    speed_cmd_cm_s = 0.0f;
    speed_cmd_accel_cm_s2 = 0.0f;

    if (enable == 0U)
    {
        Motor_Brake_All();
    }
    else
    {
        basic_speed = PID_Clamp_Speed_Target(basic_speed);
        if (basic_speed < 0)
        {
            /* Publish one tick early so Balance_Task can pre-position the rod
             * before the first nonzero wheel-speed target is applied. */
            speed_cmd_accel_cm_s2 = -SPEED_RAMP_CM_S2;
        }
        pid_running = 1U;
    }
}

/* Positional speed PD plus dead-zone feedforward. pid->out keeps the final duty
 * so live status and the anti-windup saturation test both see what the
 * motor actually received. */
static float Speed_PD_Calculate(PID_T *pid, float current,
                                const PidParams_t *params)
{
    float feedforward = 0.0f;
    float output;

    if (pid->target > 0.0f)
    {
        feedforward = speed_ff_duty;
    }
    else if (pid->target < 0.0f)
    {
        feedforward = -speed_ff_duty;
    }

    output = pid_calculate_positional_ff(pid, current, feedforward);
    output = pid_constrain(output, params->out_min, params->out_max);

    if ((output >= params->out_max) || (output <= params->out_min))
    {
        ++mon_saturated;
    }

    /* Never let the output oppose the target sign. Driving a spinning motor the
     * other way is plugging: the current can reach twice stall, and on this
     * vehicle the resulting jolt is exactly the disturbance the ball on the rod
     * cannot absorb. Coasting back down to the target is slower and correct. */
    if (((pid->target > 0.0f) && (output < 0.0f)) ||
        ((pid->target < 0.0f) && (output > 0.0f)))
    {
        output = 0.0f;
        ++mon_zero_clamp;
    }

    pid->out = output;
    return output;
}

void PID_Speed_Task(void)
{
    float output_left;
    float output_right;

    if (pid_running == 0)
    {
        return;
    }

    /* First thing in the tick: both branches below build their targets on the
     * same ramped common-mode command. */
    PID_RampCommonMode();
    PID_UpdateSpeedFeedback();

#if SPEED_PID_DEBUG_MODE
    line_speed_correction = 0.0f;
    left_speed_target = PID_Clamp_Speed_Target(PID_RoundToInt(speed_cmd_cm_s));
    right_speed_target = PID_Clamp_Speed_Target(PID_RoundToInt(speed_cmd_cm_s));
#else
    if (PID_UpdateLineTargets() == 0U)
    {
        ++pid_brake_ticks;
        left_speed_target = 0;
        right_speed_target = 0;
        pid_reset(&pid_speed_left);
        pid_reset(&pid_speed_right);
        pid_set_target(&pid_speed_left, 0.0f);
        pid_set_target(&pid_speed_right, 0.0f);
        speed_cmd_cm_s = 0.0f;
        speed_cmd_accel_cm_s2 = 0.0f;
        /* Coast, do not brake. Braking shorts the windings through the H bridge
         * and on a moving vehicle that is a hard stop drawing large current;
         * alternating it with full drive at 100 Hz is what made the wheels
         * judder. Stopping at a marked line is a deliberate act and belongs to
         * the mission logic, which can call Motor_Brake_All() when it means it. */
        Motor_Stop_All();
        return;
    }
#endif

    ++pid_drive_ticks;

    if (left_speed_target == 0)
    {
        pid_reset(&pid_speed_left);
        Motor_Stop(&left_motor);
    }
    else
    {
        pid_set_target(&pid_speed_left, (float)left_speed_target);
        output_left = Speed_PD_Calculate(&pid_speed_left,
                                         speed_feedback_left_cm_s,
                                         &pid_params_left);
        Motor_Set_Duty(&left_motor, output_left);
    }

    if (right_speed_target == 0)
    {
        pid_reset(&pid_speed_right);
        Motor_Stop(&right_motor);
    }
    else
    {
        pid_set_target(&pid_speed_right, (float)right_speed_target);
        output_right = Speed_PD_Calculate(&pid_speed_right,
                                          speed_feedback_right_cm_s,
                                          &pid_params_right);
        Motor_Set_Duty(&right_motor, output_right);
    }

    PID_MonitorAccumulate(left_encoder.speed_cm_s, right_encoder.speed_cm_s);
}

void PID_SetMonitor(uint32_t period_ms)
{
    pid_monitor_period_ms = period_ms;
    pid_monitor_last_tick = HAL_GetTick();
}

void PID_MonitorTask(void)
{
    float l_min;
    float l_max;
    float l_avg;
    float r_min;
    float r_max;
    float r_avg;
    uint16_t samples;
    uint16_t zero_clamp;
    uint16_t saturated;
    uint32_t primask;

    if (pid_monitor_period_ms == 0U)
    {
        return;
    }
    if ((HAL_GetTick() - pid_monitor_last_tick) < pid_monitor_period_ms)
    {
        return;
    }
    pid_monitor_last_tick = HAL_GetTick();

    /* The control interrupt writes these; take the whole set at once so the
     * printed envelope belongs to one interval and not to two halves. */
    primask = __get_PRIMASK();
    __disable_irq();
    samples = mon_samples;
    l_min = mon_left_min;
    l_max = mon_left_max;
    l_avg = (samples != 0U) ? (mon_left_sum / (float)samples) : 0.0f;
    r_min = mon_right_min;
    r_max = mon_right_max;
    r_avg = (samples != 0U) ? (mon_right_sum / (float)samples) : 0.0f;
    zero_clamp = mon_zero_clamp;
    saturated = mon_saturated;
    mon_samples = 0U;
    mon_zero_clamp = 0U;
    mon_saturated = 0U;
    __set_PRIMASK(primask);

    if (samples == 0U)
    {
        return;
    }

    Uart_Printf(DEBUG_UART,
                "M tgt=%d L=%.1f[%.1f..%.1f] R=%.1f[%.1f..%.1f] d=%.0f,%.0f z=%u s=%u\r\n",
                left_speed_target,
                l_avg, l_min, l_max,
                r_avg, r_min, r_max,
                pid_speed_left.out, pid_speed_right.out,
                zero_clamp, saturated);
}

void PID_Task(void)
{
    PID_Speed_Task();
}
