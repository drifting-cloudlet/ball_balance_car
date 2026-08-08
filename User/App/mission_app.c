#include "mission_app.h"
#include "MyDefine.h"
#include <math.h>

volatile uint8_t mission_enabled = 1U;
volatile uint8_t mission_cross_min_channels = MISSION_CROSS_MIN_CHANNELS;

static volatile MissionState_t mission_state = MISSION_IDLE;
static volatile MissionMode_t mission_mode = MISSION_MODE_LAP;
static volatile uint32_t mission_start_tick = 0U;
static volatile uint32_t mission_elapsed_ms = 0U;
static volatile float mission_distance_cm = 0.0f;
static volatile uint8_t mission_missed = 0U;

static int mission_cruise_speed = -45;
static uint8_t mission_cross_ticks = 0U;
static uint8_t mission_stop_ticks = 0U;
/* Written in the control interrupt, read by the status print. */
static volatile uint8_t mission_ball_dwell = 0U;
static volatile float mission_hold_target_m = MISSION_HOLD_DEFAULT_M;

static const char *Mission_ModeName(MissionMode_t mode)
{
    switch (mode)
    {
        case MISSION_MODE_HOLD: return "HOLD";
        case MISSION_MODE_BALL: return "Q3";
        case MISSION_MODE_Q4:   return "Q4";
        case MISSION_MODE_Q5:   return "Q5";
        case MISSION_MODE_Q6:   return "Q6";
        default:                return "Q2-LAP";
    }
}

void Mission_Init(void)
{
    mission_state = MISSION_IDLE;
    mission_start_tick = 0U;
    mission_elapsed_ms = 0U;
    mission_distance_cm = 0.0f;
    mission_missed = 0U;
    mission_cross_ticks = 0U;
    mission_stop_ticks = 0U;
    mission_ball_dwell = 0U;
    mission_mode = MISSION_BOOT_MODE;
    mission_hold_target_m = MISSION_HOLD_DEFAULT_M;
    mission_cruise_speed = basic_speed;

    /* Loud, because getting this wrong on a bench drives the car off the table
     * and getting it wrong on the track wastes an attempt. */
    Uart_Printf(DEBUG_UART,
                "Mission_Init MODE=%s | KEY2=Q2 | PC0/2/3/13=start/abort\r\n",
                Mission_ModeName(MISSION_BOOT_MODE));
    Uart_Printf(DEBUG_UART,
                "  lap=%.0fcm arm=%.0f approach=%.0f | Q3 %.0fmm "
                "tol=%.0fmm/%.0fmm-s dwell=%ums\r\n",
                MISSION_LAP_CM, MISSION_ARM_CM, MISSION_APPROACH_CM,
                MISSION_BALL_TARGET_M * 1000.0f,
                MISSION_BALL_TOL_M * 1000.0f,
                MISSION_BALL_VEL_M_S * 1000.0f,
                (unsigned int)(MISSION_BALL_DWELL_TICKS * 10U));
}

void Mission_SetMode(MissionMode_t mode)
{
    /* DONE may still be actively holding the final ball target. Selecting a
     * requirement must always leave a genuinely stopped RDY state. */
    if (mission_state != MISSION_IDLE)
    {
        Mission_Abort();
    }
    if (pid_running != 0U)
    {
        PID_Set_Enable(0U);
    }
    if (Balance_IsEnabled() != 0U)
    {
        Balance_SetEnable(0U);
    }

    mission_mode = mode;
    mission_state = MISSION_IDLE;
    mission_elapsed_ms = 0U;

    Uart_Printf(DEBUG_UART,
                "mission mode=%s tgt=%.1fmm | external=start/abort KEY2=Q2\r\n",
                Mission_ModeName(mode),
                mission_hold_target_m * 1000.0f);
}

MissionMode_t Mission_GetMode(void)
{
    return mission_mode;
}

void Mission_SetHoldTarget(float x_m)
{
    /* Preserve balx's existing live-tuning behavior while keeping the next Q6
     * target in one clamped source of truth. */
    Balance_SetTarget(x_m);
    mission_hold_target_m = Balance_GetTarget();
}

float Mission_GetHoldTarget(void)
{
    return mission_hold_target_m;
}

static void Mission_StartHold(void)
{
    /* Static HOLD, including Q6 pre-position, must never inherit wheel motion
     * from a previous line-following or speed-tuning run. */
    PID_Set_Enable(0U);
    mission_elapsed_ms = 0U;
    mission_distance_cm = 0.0f;
    mission_missed = 0U;

    Uart_Printf(DEBUG_UART, "mission hold start, tgt=%.1fmm x=%.1fmm%s\r\n",
                mission_hold_target_m * 1000.0f,
                Ball_GetPosition() * 1000.0f,
                (Ball_IsUsable() != 0U) ? "" : "  *** VISION LOST ***");

    Balance_SetTarget(mission_hold_target_m);
    Balance_SetEnable(1U);

    mission_start_tick = HAL_GetTick();
    mission_state = MISSION_HOLD;
}

/* Engaging the ball loop belongs here and not in Mission_Task because
 * Balance_SetEnable prints. The interrupt only ever moves the target. */
static void Mission_StartBall(uint32_t start_tick)
{
    PID_Set_Enable(0U);
    mission_ball_dwell = 0U;
    mission_elapsed_ms = 0U;
    mission_distance_cm = 0.0f;
    mission_missed = 0U;

    /* The requirement starts the ball at O. Report where it actually is instead
     * of refusing to run: a centimetre of placement error should not cost an
     * attempt in front of a judge, but it should be on the record. */
    Uart_Printf(DEBUG_UART, "mission ball start, x=%.1fmm%s\r\n",
                Ball_GetPosition() * 1000.0f,
                (Ball_IsUsable() != 0U) ? "" : "  *** VISION LOST ***");

    Balance_SetTarget(+MISSION_BALL_TARGET_M);
    Balance_SetEnable(1U);

    /* State last. Until it moves off IDLE the interrupt returns immediately, so
     * there is no window where the sequence runs against an unset clock. */
    mission_start_tick = start_tick;
    mission_state = MISSION_BALL_POS;
}

static void Mission_StartBalancedDrive(uint32_t start_tick, int cruise_speed,
                                       float target_m)
{
    uint32_t primask;

    /* Balanced requirements temporarily borrow basic_speed. Preserve the Q2
     * cruise value so abort/completion and a KEY2 handoff restore it exactly. */
    mission_cruise_speed = basic_speed;
    basic_speed = cruise_speed;
    mission_cross_ticks = 0U;
    mission_stop_ticks = 0U;
    mission_missed = 0U;
    mission_elapsed_ms = 0U;
    mission_distance_cm = 0.0f;

    Balance_SetTarget(target_m);
    if (Balance_IsEnabled() == 0U)
    {
        Balance_SetEnable(1U);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    Encoder_Reset_Total_Count();
    mission_start_tick = start_tick;
    mission_state = MISSION_RUN;
    __set_PRIMASK(primask);

    PID_Set_Enable(1U);
    Uart_Printf(DEBUG_UART,
                "%s start: hold %.1fmm, cruise %dcm/s%s\r\n",
                Mission_ModeName(mission_mode),
                Balance_GetTarget() * 1000.0f, cruise_speed,
                (Ball_IsUsable() != 0U) ? "" : "  *** VISION LOST ***");
}

static void Mission_StartLap(uint32_t start_tick)
{
    uint32_t primask;

    /* Remember what the operator set as cruise speed so the approach phase can
     * borrow basic_speed and hand it back afterwards. */
    mission_cruise_speed = basic_speed;

    mission_cross_ticks = 0U;
    mission_stop_ticks = 0U;
    mission_missed = 0U;
    mission_elapsed_ms = 0U;
    mission_distance_cm = 0.0f;

    /* Encoder_Task runs in the control interrupt and owns these counters the
     * rest of the time. Reset them with the interrupt held off so the trip does
     * not start with a partial tick already counted. */
    primask = __get_PRIMASK();
    __disable_irq();
    Encoder_Reset_Total_Count();
    mission_start_tick = start_tick;
    mission_state = MISSION_RUN;
    __set_PRIMASK(primask);

    PID_Set_Enable(1U);
    Uart_Printf(DEBUG_UART, "mission start, cruise %d cm/s\r\n",
                mission_cruise_speed);
}

void Mission_Start(void)
{
    if (mission_mode == MISSION_MODE_HOLD)
    {
        Mission_StartHold();
        return;
    }

    if (mission_mode == MISSION_MODE_BALL)
    {
        Mission_StartBall(HAL_GetTick());
        return;
    }

    if (mission_mode == MISSION_MODE_Q4)
    {
        Mission_StartBalancedDrive(HAL_GetTick(),
                                   MISSION_Q4_CRUISE_SPEED_CM_S, 0.0f);
        return;
    }

    if (mission_mode == MISSION_MODE_Q5)
    {
        Mission_StartBalancedDrive(HAL_GetTick(),
                                   MISSION_Q5_CRUISE_SPEED_CM_S, 0.0f);
        return;
    }

    if (mission_mode == MISSION_MODE_Q6)
    {
        Mission_StartBalancedDrive(HAL_GetTick(),
                                   MISSION_Q5_CRUISE_SPEED_CM_S,
                                   mission_hold_target_m);
        return;
    }

    Mission_StartLap(HAL_GetTick());
}

void Mission_Abort(void)
{
    if ((mission_mode == MISSION_MODE_BALL) ||
        (mission_mode == MISSION_MODE_HOLD) ||
        (mission_state == MISSION_HOLD))
    {
        mission_state = MISSION_IDLE;
        mission_ball_dwell = 0U;
        /* Disengaging returns the actuator target to motor zero. The car was
         * never moving in these modes, so the speed loop is left alone. */
        Balance_SetEnable(0U);
        return;
    }

    mission_state = MISSION_IDLE;
    mission_cross_ticks = 0U;
    mission_stop_ticks = 0U;
    basic_speed = mission_cruise_speed;
    PID_Set_Enable(0U);
    if ((mission_mode == MISSION_MODE_Q4) ||
        (mission_mode == MISSION_MODE_Q5) ||
        (mission_mode == MISSION_MODE_Q6))
    {
        Balance_SetEnable(0U);
    }
    Uart_Printf(DEBUG_UART, "mission aborted at %.1fcm\r\n",
                mission_distance_cm);
}

void Mission_Toggle(void)
{
    if ((mission_state == MISSION_IDLE) ||
        (mission_state == MISSION_DONE) ||
        ((mission_mode == MISSION_MODE_Q6) &&
         (mission_state == MISSION_HOLD)))
    {
        Mission_Start();
    }
    else
    {
        Mission_Abort();
    }
}

void Mission_Q2Toggle(void)
{
    uint32_t start_tick = HAL_GetTick();

    if ((mission_mode == MISSION_MODE_LAP) &&
        (mission_state != MISSION_IDLE) &&
        (mission_state != MISSION_DONE))
    {
        Mission_Abort();
        return;
    }

    if (mission_mode != MISSION_MODE_LAP)
    {
        Mission_SetMode(MISSION_MODE_LAP);
    }
    Mission_StartLap(start_tick);
}

void Mission_Q3Start(void)
{
    uint32_t start_tick = HAL_GetTick();

    Mission_SetMode(MISSION_MODE_BALL);
    /* Competition timing begins on the button edge, including all startup
     * diagnostics and controller reset work before the first motor command. */
    Mission_StartBall(start_tick);
}

void Mission_Q6HoldTarget(void)
{
    Mission_SetMode(MISSION_MODE_Q6);
    Mission_StartHold();
}

void Mission_Q3Stop(void)
{
    PID_Set_Enable(0U);
    mission_state = MISSION_IDLE;
    mission_ball_dwell = 0U;
    mission_cross_ticks = 0U;
    mission_stop_ticks = 0U;
    basic_speed = mission_cruise_speed;
    Balance_SetEnable(0U);
    Uart_Printf(DEBUG_UART, "Q3 stopped; wheels off, actuator centered\r\n");
}

void Mission_Q4Start(void)
{
    uint32_t start_tick = HAL_GetTick();

    Mission_SetMode(MISSION_MODE_Q4);
    Mission_StartBalancedDrive(start_tick, MISSION_Q4_CRUISE_SPEED_CM_S, 0.0f);
}

void Mission_Q5Start(void)
{
    uint32_t start_tick = HAL_GetTick();

    Mission_SetMode(MISSION_MODE_Q5);
    Mission_StartBalancedDrive(start_tick, MISSION_Q5_CRUISE_SPEED_CM_S, 0.0f);
}

void Mission_Q6Start(void)
{
    uint32_t start_tick = HAL_GetTick();

    Mission_SetMode(MISSION_MODE_Q6);
    Mission_StartBalancedDrive(start_tick, MISSION_Q5_CRUISE_SPEED_CM_S,
                               mission_hold_target_m);
}

void Mission_Q6Toggle(void)
{
    if ((mission_mode == MISSION_MODE_Q6) &&
        (mission_state == MISSION_HOLD))
    {
        /* Preserve the already-regulated target and keep the balance loop
         * engaged while the wheels and scored timer start. */
        Mission_Start();
        return;
    }

    if ((mission_mode == MISSION_MODE_Q6) && (mission_state != MISSION_IDLE))
    {
        Mission_Abort();
        return;
    }

    Mission_Q6Start();
}

/* The finish line is the only place on the track where several channels go dark
 * at once. Requiring the pattern to persist keeps one noisy frame from ending
 * the run, at the cost of 0.6 cm of travel at approach speed. */
static uint8_t Mission_CrossLineSeen(void)
{
    if (g_line_dark_count >= mission_cross_min_channels)
    {
        if (mission_cross_ticks < 255U)
        {
            ++mission_cross_ticks;
        }
    }
    else
    {
        mission_cross_ticks = 0U;
    }

    return (mission_cross_ticks >= MISSION_CROSS_CONFIRM_TICKS) ? 1U : 0U;
}

static void Mission_EnterBrake(void)
{
    mission_stop_ticks = 0U;
    mission_state = MISSION_BRAKE;
    /* PID_Set_Enable(0) brakes both motors. This is the one place in the
     * firmware that brakes on purpose; every other stop coasts. */
    PID_Set_Enable(0U);
}

/* Requirement 3. Shares the 10 ms interrupt with the lap logic, which is safe
 * only because the single thing it writes is the ball target: Balance_SetTarget
 * is one clamped float store. The rod remains in the main-loop
 * Ball -> Balance -> Pendulum chain; UART4 only advances prepared frames in its
 * own interrupt, and Balance_SetEnable still prints. */
static void Mission_BallTask(void)
{
    if (Ball_IsUsable() == 0U)
    {
        /* No trustworthy position, so no progress. balance_app has already
         * levelled the rod. The clock keeps running and the run is not aborted:
         * the 5 s limit reports the failure without needing a second mechanism
         * that decides what "failed" means. */
        mission_ball_dwell = 0U;
        return;
    }

    /* Compare against the target the loop is actually driving to, not against a
     * copy of it. One owner per number. */
    if ((fabsf(Balance_GetTarget() - Ball_GetPosition()) > MISSION_BALL_TOL_M) ||
        (fabsf(Ball_GetVelocity()) > MISSION_BALL_VEL_M_S))
    {
        mission_ball_dwell = 0U;
        return;
    }

    if (++mission_ball_dwell < MISSION_BALL_DWELL_TICKS)
    {
        return;
    }

    mission_ball_dwell = 0U;

    if (mission_state == MISSION_BALL_POS)
    {
        Balance_SetTarget(-MISSION_BALL_TARGET_M);
        mission_state = MISSION_BALL_NEG;
        return;
    }

    /* Settled at -5 cm. Freeze the clock but leave the loop engaged: the
     * requirement is that the ball stays there, not that the controller lets go
     * of it the moment the stopwatch stops. */
    mission_elapsed_ms = HAL_GetTick() - mission_start_tick;
    mission_state = MISSION_DONE;
}

void Mission_Task(void)
{
    float distance;

    if ((mission_state == MISSION_IDLE) ||
        (mission_state == MISSION_DONE) ||
        (mission_state == MISSION_Q5_PASSED))
    {
        return;
    }

    if ((mission_mode == MISSION_MODE_HOLD) ||
        (mission_state == MISSION_HOLD))
    {
        /* Nothing to sequence. balance_app owns the loop and commands motor zero
         * when vision drops, so there is no second opinion to add here.
         * The run ends when the operator ends it. */
        return;
    }

    if (mission_mode == MISSION_MODE_BALL)
    {
        Mission_BallTask();
        return;
    }

    distance = Encoder_Get_Average_Distance_Cm();
    mission_distance_cm = distance;

    if (mission_mode == MISSION_MODE_Q4)
    {
        if (mission_state == MISSION_BRAKE)
        {
            basic_speed = 0;

            if ((fabsf(speed_cmd_cm_s) <= 0.01f) &&
                (fabsf(left_encoder.speed_cm_s) < MISSION_STOP_SPEED_CM_S) &&
                (fabsf(right_encoder.speed_cm_s) < MISSION_STOP_SPEED_CM_S))
            {
                if (++mission_stop_ticks >= MISSION_STOP_CONFIRM_TICKS)
                {
                    /* The H bridge only locks wheels that are already stopped.
                     * The balance loop remains enabled and keeps holding O. */
                    PID_Set_Enable(0U);
                    mission_elapsed_ms = HAL_GetTick() - mission_start_tick;
                    mission_state = MISSION_DONE;
                    basic_speed = mission_cruise_speed;
                }
            }
            else
            {
                mission_stop_ticks = 0U;
            }
            return;
        }

        if (distance >= MISSION_Q4_DECEL_START_CM)
        {
            basic_speed = 0;
            mission_stop_ticks = 0U;
            mission_state = MISSION_BRAKE;
        }
        return;
    }

    if ((mission_mode == MISSION_MODE_Q5) ||
        (mission_mode == MISSION_MODE_Q6))
    {
        if ((distance >= MISSION_ARM_CM) && (Mission_CrossLineSeen() != 0U))
        {
            /* Q5 is scored when the car passes A. Freeze the display clock but
             * leave both control loops running; no stop is requested. */
            mission_elapsed_ms = HAL_GetTick() - mission_start_tick;
            mission_state = MISSION_Q5_PASSED;
            return;
        }

        if (distance >= MISSION_MAX_CM)
        {
            /* The car has physically passed A even if the perpendicular-line
             * pattern was missed. Preserve a finite diagnostic time and keep
             * driving so the fallback cannot introduce an unrequested brake. */
            mission_missed = 1U;
            mission_elapsed_ms = HAL_GetTick() - mission_start_tick;
            mission_state = MISSION_Q5_PASSED;
        }
        return;
    }

    if (mission_state == MISSION_BRAKE)
    {
        if ((fabsf(left_encoder.speed_cm_s) < MISSION_STOP_SPEED_CM_S) &&
            (fabsf(right_encoder.speed_cm_s) < MISSION_STOP_SPEED_CM_S))
        {
            if (++mission_stop_ticks >= MISSION_STOP_CONFIRM_TICKS)
            {
                /* Freeze the clock when the car has actually stopped rather
                 * than when the line was seen. The difference is about 0.2 s,
                 * and a display that stops while the car is still rolling
                 * invites the obvious question from a judge. */
                mission_elapsed_ms = HAL_GetTick() - mission_start_tick;
                mission_state = MISSION_DONE;
                basic_speed = mission_cruise_speed;
            }
        }
        else
        {
            mission_stop_ticks = 0U;
        }
        return;
    }

    if ((mission_state == MISSION_RUN) && (distance >= MISSION_APPROACH_CM))
    {
        basic_speed = MISSION_APPROACH_SPEED_CM_S;
        mission_state = MISSION_APPROACH;
    }

    if ((distance >= MISSION_ARM_CM) && (Mission_CrossLineSeen() != 0U))
    {
        Mission_EnterBrake();
        return;
    }

    if (distance >= MISSION_MAX_CM)
    {
        mission_missed = 1U;
        Mission_EnterBrake();
    }
}

MissionState_t Mission_GetState(void)
{
    return mission_state;
}

uint32_t Mission_GetElapsedMs(void)
{
    if ((mission_state == MISSION_IDLE) ||
        (mission_state == MISSION_DONE) ||
        (mission_state == MISSION_Q5_PASSED) ||
        ((mission_mode == MISSION_MODE_Q6) &&
         (mission_state == MISSION_HOLD)))
    {
        return mission_elapsed_ms;
    }

    return HAL_GetTick() - mission_start_tick;
}

float Mission_GetDistanceCm(void)
{
    return mission_distance_cm;
}

uint8_t Mission_LineWasMissed(void)
{
    return mission_missed;
}

void Mission_PrintStatus(void)
{
    static const char *const name[] = {
        "IDLE", "RUN", "APPROACH", "BRAKE", "DONE", "BALL+5", "BALL-5", "HOLD",
        "PASSED"
    };
    uint32_t ms = Mission_GetElapsedMs();

    if ((mission_mode == MISSION_MODE_BALL) || (mission_mode == MISSION_MODE_HOLD))
    {
        Uart_Printf(DEBUG_UART,
                    "mis %s %s t=%lu.%02lus x=%.1f tgt=%.0f v=%.1fmm/s dwell=%u\r\n",
                    Mission_ModeName(mission_mode),
                    name[mission_state],
                    (unsigned long)(ms / 1000U),
                    (unsigned long)((ms % 1000U) / 10U),
                    Ball_GetPosition() * 1000.0f,
                    Balance_GetTarget() * 1000.0f,
                    Ball_GetVelocity() * 1000.0f,
                    (unsigned int)mission_ball_dwell);
        return;
    }

    Uart_Printf(DEBUG_UART,
                "mis %s %s t=%lu.%02lus d=%.1fcm dark=%u/%u%s\r\n",
                Mission_ModeName(mission_mode),
                name[mission_state],
                (unsigned long)(ms / 1000U),
                (unsigned long)((ms % 1000U) / 10U),
                mission_distance_cm,
                (unsigned int)g_line_dark_count,
                (unsigned int)mission_cross_min_channels,
                (mission_missed != 0U) ? " LINE MISSED" : "");
}
