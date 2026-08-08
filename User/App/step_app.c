#include "step_app.h"

static void Step_SplitRPM(int16_t rpm, uint8_t *direction, uint16_t *velocity)
{
    int32_t signed_rpm = rpm;
    uint32_t magnitude;

    if (signed_rpm >= 0)
    {
        *direction = 0U;
        magnitude = (uint32_t)signed_rpm;
    }
    else
    {
        *direction = 1U;
        magnitude = (uint32_t)(-signed_rpm);
    }

    if (magnitude > STEP_PROTOCOL_MAX_RPM)
    {
        magnitude = STEP_PROTOCOL_MAX_RPM;
    }
    *velocity = (uint16_t)magnitude;
}

void Step_Init(void)
{
    Emm_V5_En_Control(STEP_A_UART, STEP_A_ADDR, true, STEP_SYNC);
    Emm_V5_En_Control(STEP_B_UART, STEP_B_ADDR, true, STEP_SYNC);
    Step_Stop();
}

void Step_SetRPM(int16_t a_rpm, int16_t b_rpm)
{
    uint8_t a_direction;
    uint8_t b_direction;
    uint16_t a_velocity;
    uint16_t b_velocity;

    Step_SplitRPM(a_rpm, &a_direction, &a_velocity);
    Step_SplitRPM(b_rpm, &b_direction, &b_velocity);
    Emm_V5_Vel_Control(STEP_A_UART, STEP_A_ADDR, a_direction, a_velocity,
                       STEP_ACCELERATION, STEP_SYNC);
    Emm_V5_Vel_Control(STEP_B_UART, STEP_B_ADDR, b_direction, b_velocity,
                       STEP_ACCELERATION, STEP_SYNC);
}

void Step_SetSpeed(int8_t a_percent, int8_t b_percent)
{
    if (a_percent > 100) a_percent = 100;
    if (a_percent < -100) a_percent = -100;
    if (b_percent > 100) b_percent = 100;
    if (b_percent < -100) b_percent = -100;

    Step_SetRPM((int16_t)(((int32_t)a_percent * STEP_MAX_RPM) / 100),
                (int16_t)(((int32_t)b_percent * STEP_MAX_RPM) / 100));
}

void Step_Stop(void)
{
    Emm_V5_Stop_Now(STEP_A_UART, STEP_A_ADDR, STEP_SYNC);
    Emm_V5_Stop_Now(STEP_B_UART, STEP_B_ADDR, STEP_SYNC);
}

void Step_Disable(void)
{
    Emm_V5_En_Control(STEP_A_UART, STEP_A_ADDR, false, STEP_SYNC);
    Emm_V5_En_Control(STEP_B_UART, STEP_B_ADDR, false, STEP_SYNC);
}

void Step_EnableOne(UART_HandleTypeDef *uart, uint8_t addr, bool enable)
{
    Emm_V5_En_Control(uart, addr, enable, STEP_SYNC);
}

void Step_StopOne(UART_HandleTypeDef *uart, uint8_t addr)
{
    Emm_V5_Stop_Now(uart, addr, STEP_SYNC);
}

void Step_MoveAngle(UART_HandleTypeDef *uart, uint8_t addr, float degrees,
                    uint16_t rpm, uint8_t acceleration, bool absolute)
{
    uint8_t direction = 0U;
    uint32_t pulses;

    if (degrees < 0.0f)
    {
        direction = 1U;
        degrees = -degrees;
    }
    if (rpm > STEP_PROTOCOL_MAX_RPM)
    {
        rpm = STEP_PROTOCOL_MAX_RPM;
    }

    pulses = (uint32_t)(degrees * ((float)STEP_PULSES_PER_REV / 360.0f) + 0.5f);
    Emm_V5_Pos_Control(uart, addr, direction, rpm, acceleration, pulses,
                       absolute, STEP_SYNC);
}

void Step_SetZeroHere(UART_HandleTypeDef *uart, uint8_t addr)
{
    Emm_V5_Reset_CurPos_To_Zero(uart, addr);
}

void Step_Home(UART_HandleTypeDef *uart, uint8_t addr, uint8_t mode)
{
    Emm_V5_Origin_Trigger_Return(uart, addr, mode, STEP_SYNC);
}
