#include "pendulum_app.h"
#include "uart_driver.h"
#include <math.h>

#define PEND_MOTOR_SLEW_PER_TICK \
    (PEND_MOTOR_SLEW_DEG_S * ((float)PEND_TASK_PERIOD_MS / 1000.0f))

static float pendulum_target_motor_deg = 0.0f;
static float pendulum_applied_motor_deg = 0.0f;
static int32_t pendulum_last_pulses = 0;
static uint8_t pendulum_pulses_valid = 0U;
static uint8_t pendulum_energised = 0U;

static float Pendulum_Clamp(float value, float limit)
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

static float Pendulum_ClampRange(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static int32_t Pendulum_MotorDegToPulses(float motor_deg)
{
    float pulses = motor_deg * ((float)STEP_PULSES_PER_REV / 360.0f);

    return (pulses >= 0.0f) ? (int32_t)(pulses + 0.5f)
                            : (int32_t)(pulses - 0.5f);
}

static void Pendulum_SendMotor(float motor_deg)
{
    int32_t pulses = Pendulum_MotorDegToPulses(motor_deg);
    int32_t delta;

    if (pendulum_pulses_valid != 0U)
    {
        delta = pulses - pendulum_last_pulses;
        if ((delta >= -PEND_PULSE_DEADBAND) && (delta <= PEND_PULSE_DEADBAND))
        {
            return;
        }
    }

    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_deg,
                   PEND_SERVO_RPM, PEND_SERVO_ACC, true);
    pendulum_last_pulses = pulses;
    pendulum_pulses_valid = 1U;
}

void Pendulum_SetEnergised(uint8_t on)
{
    if (on != 0U)
    {
        /* Close the driver's position loop first, then adopt the held shaft
         * position as zero. This is the command order accepted reliably by the
         * installed ZDT driver; enabling alone does not issue a move. */
        Step_EnableOne(STEP_A_UART, STEP_A_ADDR, true);
        Step_SetZeroHere(STEP_A_UART, STEP_A_ADDR);
        pendulum_target_motor_deg = 0.0f;
        pendulum_applied_motor_deg = 0.0f;
        pendulum_last_pulses = 0;
        pendulum_pulses_valid = 1U;
        pendulum_energised = 1U;
    }
    else
    {
        Step_EnableOne(STEP_A_UART, STEP_A_ADDR, false);
        pendulum_energised = 0U;
    }
}

uint8_t Pendulum_IsEnergised(void)
{
    return pendulum_energised;
}

void Pendulum_Init(void)
{
    pendulum_target_motor_deg = 0.0f;
    pendulum_applied_motor_deg = 0.0f;

#if PENDULUM_ENERGISE_AT_BOOT
    /* Close the loop at the present shaft position, then latch it as zero
     * before any absolute target is issued. */
    Step_EnableOne(STEP_A_UART, STEP_A_ADDR, true);
    Step_SetZeroHere(STEP_A_UART, STEP_A_ADDR);
    pendulum_target_motor_deg = PEND_MOTOR_CENTER_DEG;
#else
    Step_EnableOne(STEP_A_UART, STEP_A_ADDR, false);
#endif

    pendulum_last_pulses = 0;
    pendulum_pulses_valid = 1U;
    pendulum_energised = (PENDULUM_ENERGISE_AT_BOOT != 0) ? 1U : 0U;

    Uart_Printf(DEBUG_UART,
                "Pendulum_Init center=%.1fdeg range=[%.1f,%.1f] slew=%.0fdeg/s "
                "rpm=%u acc=%u, pendulum %s\r\n",
                PEND_MOTOR_CENTER_DEG, PEND_MOTOR_MIN_DEG,
                PEND_MOTOR_MAX_DEG, PEND_MOTOR_SLEW_DEG_S,
                (unsigned int)PEND_SERVO_RPM,
                (unsigned int)PEND_SERVO_ACC,
                (pendulum_energised != 0U) ? "ENERGISED" : "off");
}

void Pendulum_SetMotorAngle(float motor_deg)
{
    pendulum_target_motor_deg = Pendulum_ClampRange(motor_deg,
                                                    PEND_MOTOR_MIN_DEG,
                                                    PEND_MOTOR_MAX_DEG);
}

void Pendulum_Center(void)
{
    pendulum_target_motor_deg = PEND_MOTOR_CENTER_DEG;
}

void Pendulum_Task(void)
{
    float step;

    if (pendulum_energised == 0U)
    {
        return;
    }

    step = pendulum_target_motor_deg - pendulum_applied_motor_deg;
    step = Pendulum_Clamp(step, PEND_MOTOR_SLEW_PER_TICK);
    pendulum_applied_motor_deg += step;

    Pendulum_SendMotor(pendulum_applied_motor_deg);
}

float Pendulum_GetTargetMotorDeg(void)
{
    return pendulum_target_motor_deg;
}

float Pendulum_GetAppliedMotorDeg(void)
{
    return pendulum_applied_motor_deg;
}

void Pendulum_ZeroHere(void)
{
    Step_SetZeroHere(STEP_A_UART, STEP_A_ADDR);
    pendulum_target_motor_deg = 0.0f;
    pendulum_applied_motor_deg = 0.0f;
    pendulum_last_pulses = 0;
    pendulum_pulses_valid = 1U;
    Uart_Printf(DEBUG_UART, "pendulum zero latched at current motor pos\r\n");
}

#if PENDULUM_PROBE_MODE

#define PROBE_OVERRIDE_RPM      30U

void Pendulum_ProbeOverride(void)
{
#if PENDULUM_LINKAGE_ATTACHED
    const float motor_high_deg = PEND_MOTOR_MAX_DEG;
    const float motor_low_deg = PEND_MOTOR_MIN_DEG;
    const float motor_center_deg = PEND_MOTOR_CENTER_DEG;
    const float travel_deg = PEND_MOTOR_LIMIT_DEG;
#else
    const float motor_high_deg = 180.0f;
    const float motor_low_deg = -180.0f;
    const float motor_center_deg = 0.0f;
    const float travel_deg = 180.0f;
#endif
    const uint32_t travel_ms = (uint32_t)((travel_deg / 180.0f) * 1000.0f);
    const uint32_t cut_ms = (travel_ms * 3U) / 10U;
    uint32_t start;

    Uart_Printf(DEBUG_UART,
                "probe override: go %.1fdeg (takes %ums), interrupt at %ums "
                "with %.1fdeg\r\n",
                motor_high_deg, travel_ms, cut_ms, motor_low_deg);

    start = HAL_GetTick();
    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_high_deg,
                   PROBE_OVERRIDE_RPM, 0U, true);
    while ((HAL_GetTick() - start) < cut_ms)
    {
    }
    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_low_deg,
                   PROBE_OVERRIDE_RPM, 0U, true);

    HAL_Delay((travel_ms * 3U) + 500U);
    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_center_deg,
                   PROBE_OVERRIDE_RPM, 0U, true);
    HAL_Delay(travel_ms + 300U);
    Uart_Printf(DEBUG_UART, "probe override done, shaft back at center\r\n");
    pendulum_pulses_valid = 0U;
}

#define PROBE_SWEEP_AMP_DEG     PEND_MOTOR_LIMIT_DEG
#define PROBE_SWEEP_PERIOD_MS   2000U
#define PROBE_SWEEP_TOTAL_MS    30000U

void Pendulum_ProbeSweep(void)
{
    uint32_t start = HAL_GetTick();
    uint32_t next = start;
    uint32_t frames = 0U;

    Uart_Printf(DEBUG_UART,
                "probe sweep: center=%.1f +-%.1fdeg, %ums period, %ums total, "
                "%uHz update\r\n",
                PEND_MOTOR_CENTER_DEG, PROBE_SWEEP_AMP_DEG,
                PROBE_SWEEP_PERIOD_MS,
                PROBE_SWEEP_TOTAL_MS, 1000U / PEND_TASK_PERIOD_MS);

    while ((HAL_GetTick() - start) < PROBE_SWEEP_TOTAL_MS)
    {
        float phase;
        float motor_deg;

        if ((HAL_GetTick() - next) >= PEND_TASK_PERIOD_MS)
        {
            next += PEND_TASK_PERIOD_MS;
            phase = 6.2831853f * (float)((HAL_GetTick() - start) %
                                         PROBE_SWEEP_PERIOD_MS) /
                    (float)PROBE_SWEEP_PERIOD_MS;
            motor_deg = PEND_MOTOR_CENTER_DEG +
                        (PROBE_SWEEP_AMP_DEG * sinf(phase));
            Pendulum_SendMotor(motor_deg);
            ++frames;
        }
    }

    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, PEND_MOTOR_CENTER_DEG,
                   PEND_SERVO_RPM, PEND_SERVO_ACC, true);
    Uart_Printf(DEBUG_UART,
                "probe sweep done, %u commands sent (expected %u)\r\n",
                frames, PROBE_SWEEP_TOTAL_MS / PEND_TASK_PERIOD_MS);
    pendulum_pulses_valid = 0U;
}

#define PROBE_REPEAT_CYCLES     10U
#define PROBE_REPEAT_DWELL_MS   700U

void Pendulum_ProbeRepeat(void)
{
#if PENDULUM_LINKAGE_ATTACHED
    const float motor_max = PEND_MOTOR_MAX_DEG;
    const float motor_min = PEND_MOTOR_MIN_DEG;
    const float motor_center = PEND_MOTOR_CENTER_DEG;
#else
    const float motor_max = 180.0f;
    const float motor_min = -180.0f;
    const float motor_center = 0.0f;
#endif
    uint8_t cycle;

    Uart_Printf(DEBUG_UART,
                "probe repeat: %.1f -> %.1f -> %.1f -> %.1f, %u cycles\r\n",
                motor_center, motor_max, motor_min, motor_center,
                PROBE_REPEAT_CYCLES);

    for (cycle = 0U; cycle < PROBE_REPEAT_CYCLES; ++cycle)
    {
        Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_max,
                       PEND_SERVO_RPM, PEND_SERVO_ACC, true);
        HAL_Delay(PROBE_REPEAT_DWELL_MS);
        Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_min,
                       PEND_SERVO_RPM, PEND_SERVO_ACC, true);
        HAL_Delay(PROBE_REPEAT_DWELL_MS);
    }

    Step_MoveAngle(STEP_A_UART, STEP_A_ADDR, motor_center,
                   PEND_SERVO_RPM, PEND_SERVO_ACC, true);
    HAL_Delay(PROBE_REPEAT_DWELL_MS);
    Uart_Printf(DEBUG_UART,
                "probe repeat done, check the marker against its start "
                "position: any drift means lost steps or shaft slip\r\n");
    pendulum_pulses_valid = 0U;
}

#endif /* PENDULUM_PROBE_MODE */
