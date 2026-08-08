#include "gyroscope_app.h"

#include "uart_driver.h"

#define GYROSCOPE_SAMPLE_DT_DEFAULT 0.01f

uint8_t first_gyroscope_flag = 0;
volatile uint32_t g_yaw_rate_tick = 0U;
static Vector3f gyroscope_gyro = {0};
static Vector3f gyroscope_accel = {0};
static float gyroscope_temperature = 0.0f;
static uint32_t gyroscope_last_tick = 0U;
static uint8_t gyroscope_initialized = 0U;

static float Gyroscope_Get_Delta_Time(void)
{
    uint32_t current_tick = HAL_GetTick();
    float dt;

    if (gyroscope_last_tick == 0U)
    {
        gyroscope_last_tick = current_tick;
        return GYROSCOPE_SAMPLE_DT_DEFAULT;
    }

    dt = (float)(current_tick - gyroscope_last_tick) / 1000.0f;
    gyroscope_last_tick = current_tick;

    if ((dt < 0.001f) || (dt > 0.1f))
    {
        dt = GYROSCOPE_SAMPLE_DT_DEFAULT;
    }

    return dt;
}

void Gyroscope_Init(void)
{
    Uart_Printf(DEBUG_UART, "ICM20608 Gyroscope_Init ......\r\n");
    first_gyroscope_flag = 0;
    gyroscope_last_tick = 0U;
    gyroscope_initialized = 0U;

    Gyroscope_Driver_Init();
    gyroscope_initialized = (ICM206xx_Init() == 0U) ? 1U : 0U;
    if (gyroscope_initialized != 0U)
    {
        Gyroscope_Calibrate_Start();
    }

    Uart_Printf(DEBUG_UART,
                "ICM20608_Init %s id=0x%02X\r\n",
                (gyroscope_initialized != 0U) ? "OK" : "FAIL",
                (unsigned int)IMU_ID);
}

void Gyroscope_Task(void)
{
    if (gyroscope_initialized == 0U)
    {
        return;
    }

    ICM206xx_Read_Data(&gyroscope_gyro, &gyroscope_accel, &gyroscope_temperature);
    Gyroscope_Update_Euler(&gyroscope_gyro, &gyroscope_accel, Gyroscope_Get_Delta_Time());

    /* The corrected rate itself is published by the driver, inside
     * Gyroscope_Update_Euler where it is already computed. Only the sample time
     * belongs here, because only this layer knows when the read happened. */
    g_yaw_rate_tick = HAL_GetTick();

    if (!first_gyroscope_flag && euler_angles.calibrated)
    {
        first_gyroscope_flag = 1;
    }
}

uint8_t Gyroscope_Rate_Is_Usable(uint32_t max_age_ms)
{
    return ((gyroscope_initialized != 0U) &&
            (euler_angles.calibrated != 0U) &&
            ((HAL_GetTick() - g_yaw_rate_tick) <= max_age_ms)) ? 1U : 0U;
}
