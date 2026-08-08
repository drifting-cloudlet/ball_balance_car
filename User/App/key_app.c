#include "key_app.h"

uint8_t key_val = 0U;
uint8_t key_old = 0U;
uint8_t key_down = 0U;
uint8_t key_up = 0U;

#if !PENDULUM_PROBE_MODE
#define KEY2_PRESS_GUARD_MS 100U
#define MISSION_KEY_PRESS_GUARD_MS 100U

static uint32_t key2_last_press_tick = 0U;
static uint8_t key2_press_seen = 0U;
static uint8_t mission_key_old = 0U;
static uint8_t mission_key_press_seen = 0U;
static uint32_t mission_key_last_press_tick[5] = {0U, 0U, 0U, 0U, 0U};

static uint8_t Key_AcceptMissionPress(uint8_t mask, uint8_t index)
{
    uint32_t now = HAL_GetTick();

    if (((mission_key_press_seen & mask) != 0U) &&
        ((now - mission_key_last_press_tick[index]) < MISSION_KEY_PRESS_GUARD_MS))
    {
        return 0U;
    }

    mission_key_press_seen |= mask;
    mission_key_last_press_tick[index] = now;
    return 1U;
}

static void Key_ToggleMission(MissionMode_t mode)
{
    if (mode == MISSION_MODE_Q6)
    {
        Mission_Q6Toggle();
        return;
    }

    if ((Mission_GetMode() == mode) &&
        (Mission_GetState() != MISSION_IDLE))
    {
        Mission_Abort();
        return;
    }

    switch (mode)
    {
        case MISSION_MODE_BALL:
            Mission_Q3Start();
            break;

        case MISSION_MODE_Q4:
            Mission_Q4Start();
            break;

        case MISSION_MODE_Q5:
            Mission_Q5Start();
            break;

        default:
            break;
    }
}

static void Key_HandleMissionPress(uint8_t pressed)
{
    /* PC1 remains a wheel-stopped Q6 pre-position action. Every other external
     * key selects and starts its requirement; pressing that key again aborts. */
    if ((pressed & Q6_BUTTON_HOLD_MASK) != 0U)
    {
        if (Key_AcceptMissionPress(Q6_BUTTON_HOLD_MASK, 1U) != 0U)
        {
            Mission_Q6HoldTarget();
        }
        return;
    }

    if ((pressed & Q6_BUTTON_TOGGLE_MASK) != 0U)
    {
        if (Key_AcceptMissionPress(Q6_BUTTON_TOGGLE_MASK, 4U) != 0U)
        {
            Key_ToggleMission(MISSION_MODE_Q6);
        }
        return;
    }

    if ((pressed & Q5_BUTTON_START_MASK) != 0U)
    {
        if (Key_AcceptMissionPress(Q5_BUTTON_START_MASK, 3U) != 0U)
        {
            Key_ToggleMission(MISSION_MODE_Q5);
        }
        return;
    }

    if ((pressed & Q4_BUTTON_START_MASK) != 0U)
    {
        if (Key_AcceptMissionPress(Q4_BUTTON_START_MASK, 2U) != 0U)
        {
            Key_ToggleMission(MISSION_MODE_Q4);
        }
        return;
    }

    if ((pressed & Q3_BUTTON_START_MASK) != 0U)
    {
        if (Key_AcceptMissionPress(Q3_BUTTON_START_MASK, 0U) != 0U)
        {
            Key_ToggleMission(MISSION_MODE_BALL);
        }
    }
}
#endif

#if PENDULUM_PROBE_MODE

/* ZDT bring-up build: the four keys drive the pendulum probes instead of their
 * normal bindings. Set PENDULUM_PROBE_MODE to 0 in pendulum_app.h to restore
 * them. */
static void Key_Handle_Press(uint8_t key_id)
{
    /* Every probe blocks the main loop for seconds at a time. Stop the wheels
     * first so a stalled scheduler cannot leave the car driving. */
    PID_Set_Enable(0U);

    switch (key_id)
    {
        case USER_BUTTON_1:
            Pendulum_ZeroHere();
            break;

        case USER_BUTTON_2:
            Pendulum_ProbeOverride();
            break;

        case USER_BUTTON_3:
            Pendulum_ProbeSweep();
            break;

        case USER_BUTTON_4:
            Pendulum_ProbeRepeat();
            break;

        default:
            break;
    }
}

#else

static void Key_Handle_Press(uint8_t key_id)
{
    switch (key_id)
    {
        case USER_BUTTON_1:
            if (Balance_IsEnabled() != 0U)
            {
                Uart_Printf(DEBUG_UART,
                            "KEY1 ignored: stop ball balance before calibration\r\n");
                break;
            }
            PID_Set_Enable(0U);
            (void)Gray_CalibrateStep();
            break;

        case USER_BUTTON_2:
            if ((key2_press_seen != 0U) &&
                ((HAL_GetTick() - key2_last_press_tick) < KEY2_PRESS_GUARD_MS))
            {
                break;
            }
            key2_press_seen = 1U;
            key2_last_press_tick = HAL_GetTick();

            /* In the competition build KEY2 belongs only to requirement 2.
             * mis:0 preserves the explicit bench-only plain PID override. */
            if (mission_enabled != 0U)
            {
                Mission_Q2Toggle();
                break;
            }

            PID_Set_Enable((pid_running == 0U) ? 1U : 0U);
#if SPEED_PID_DEBUG_MODE
            Uart_Printf(DEBUG_UART,
                        "speed pid debug %s target=%d,%d cm/s\r\n",
                        (pid_running != 0U) ? "enabled" : "disabled",
                        left_speed_target,
                        right_speed_target);
#else
            Uart_Printf(DEBUG_UART,
                        "line and speed pid %s\r\n",
                        (pid_running != 0U) ? "enabled" : "disabled");
#endif
            break;

        case USER_BUTTON_3:
            break;

        case USER_BUTTON_4:
            break;

        default:
            break;
    }
}

#endif /* PENDULUM_PROBE_MODE */

void Key_Init(void)
{
    key_val = Key_Read();
    key_old = key_val;
    key_down = 0U;
    key_up = 0U;
#if !PENDULUM_PROBE_MODE
    key2_press_seen = 0U;
    key2_last_press_tick = 0U;
    mission_key_old = Key_ReadMissionButtons();
    mission_key_press_seen = 0U;
    mission_key_last_press_tick[0] = 0U;
    mission_key_last_press_tick[1] = 0U;
    mission_key_last_press_tick[2] = 0U;
    mission_key_last_press_tick[3] = 0U;
    mission_key_last_press_tick[4] = 0U;
#endif
    Uart_Printf(DEBUG_UART, "Key_Init ......\r\n");
}

void Key_Task(void)
{
#if !PENDULUM_PROBE_MODE
    uint8_t mission_key_val;
    uint8_t mission_key_down;
#endif

    key_val = Key_Read();
    key_down = ((key_val != 0U) && (key_val != key_old)) ? key_val : 0U;
    key_up = ((key_old != 0U) && (key_val != key_old)) ? key_old : 0U;
    key_old = key_val;

#if !PENDULUM_PROBE_MODE
    mission_key_val = Key_ReadMissionButtons();
    mission_key_down = mission_key_val & (uint8_t)(~mission_key_old);
    mission_key_old = mission_key_val;

    if (mission_key_down != 0U)
    {
        Key_HandleMissionPress(mission_key_down);
    }
#endif

    if (key_down != 0U)
    {
        Key_Handle_Press(key_down);
    }
}
