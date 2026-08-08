#include "gray_app.h"
#include "oled_driver.h"
#include "uart_driver.h"
#include <string.h>

#define GRAY_LINE_THRESHOLD_RATIO     0.50f
#define GRAY_MIN_DARKNESS_RATIO       0.05f
#define GRAY_LPF_ALPHA                0.50f
#define GRAY_POSITION_LIMIT           400.0f
#define GRAY_ARRAY_SIGN               (+1.0f)
#define GRAY_LINE_ACQUIRE_TICKS       2U
#define GRAY_LINE_LOST_TICKS          3U
#define GRAY_CALIBRATION_FRAME_COUNT  10U
#define GRAY_CALIBRATION_INTERVAL_MS  10U
#define GRAY_CALIBRATION_HOLD_MS      800U

uint16_t Anolog[GRAY_CHANNEL_COUNT] = {0U};
uint16_t Normal[GRAY_CHANNEL_COUNT] = {0U};
uint8_t TrackN = 0U;
volatile float g_line_position_error = 0.0f;
volatile uint8_t g_line_detected = 0U;

/* Number of channels darker than the line threshold in the current frame.
 *
 * The track line is 1.8 cm wide and covers two channels at a 1.2 cm pitch. The
 * start/stop line at A is 5 cm long and perpendicular, so crossing it darkens
 * about four. That difference is the only thing that distinguishes "on the
 * track" from "on the finish line", so the count is exported rather than
 * recomputed from TrackN: TrackN comes from the hysteretic digital threshold
 * and can lag a fast crossing by a frame or two. */
volatile uint8_t g_line_dark_count = 0U;

static const float gray_weight[GRAY_CHANNEL_COUNT] = {
    -350.0f, -250.0f, -150.0f, -50.0f,
      50.0f,  150.0f,  250.0f, 350.0f,
};

static volatile uint8_t gray_online = 0U;
static volatile uint32_t gray_last_update_tick = 0U;
static volatile uint8_t gray_calibrating = 0U;
static uint8_t gray_line_acquire_ticks = 0U;
static uint8_t gray_line_lost_ticks = 0U;
static GrayCalibrationStage_t gray_calibration_stage = GRAY_CALIBRATION_WAIT_WHITE;
static uint16_t gray_calibration_white[GRAY_CHANNEL_COUNT] = {0U};
static uint16_t gray_calibration_black[GRAY_CHANNEL_COUNT] = {0U};

static HAL_StatusTypeDef Gray_SampleAverage(uint16_t *destination)
{
    uint32_t sum[GRAY_CHANNEL_COUNT] = {0U};
    uint8_t frame;
    uint8_t channel;

    if (destination == NULL)
    {
        return HAL_ERROR;
    }

    for (frame = 0U; frame < GRAY_CALIBRATION_FRAME_COUNT; ++frame)
    {
        if (Grayscale_ReadAll() != HAL_OK)
        {
            gray_online = 0U;
            return HAL_ERROR;
        }

        for (channel = 0U; channel < GRAY_CHANNEL_COUNT; ++channel)
        {
            sum[channel] += gray_raw[channel];
        }

        if ((frame + 1U) < GRAY_CALIBRATION_FRAME_COUNT)
        {
            HAL_Delay(GRAY_CALIBRATION_INTERVAL_MS);
        }
    }

    for (channel = 0U; channel < GRAY_CHANNEL_COUNT; ++channel)
    {
        destination[channel] =
            (uint16_t)(sum[channel] / GRAY_CALIBRATION_FRAME_COUNT);
    }

    memcpy(Anolog, gray_raw, sizeof(Anolog));
    memcpy(Normal, gray_normalized, sizeof(Normal));
    gray_online = 1U;
    gray_last_update_tick = HAL_GetTick();
    return HAL_OK;
}

static void Gray_PrintCalibration(const char *surface, const uint16_t *values)
{
    Uart_Printf(DEBUG_UART,
                "gray calibration %s=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                surface,
                values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7]);
}

static uint8_t Gray_CalculatePosition(float *position)
{
    const float threshold = (float)GRAY_ADC_FULL_SCALE * GRAY_LINE_THRESHOLD_RATIO;
    const float minimum_darkness = (float)GRAY_ADC_FULL_SCALE * GRAY_MIN_DARKNESS_RATIO;
    uint8_t valid_mask = Grayscale_GetValidMask();
    float weighted_sum = 0.0f;
    float darkness_sum = 0.0f;
    uint8_t dark_count = 0U;
    uint8_t channel;

    for (channel = 0U; channel < GRAY_CHANNEL_COUNT; ++channel)
    {
        float darkness;

        if ((valid_mask & (uint8_t)(1U << channel)) == 0U)
        {
            continue;
        }

        darkness = threshold - (float)Normal[channel];
        if (darkness <= 0.0f)
        {
            continue;
        }

        weighted_sum += gray_weight[channel] * darkness;
        darkness_sum += darkness;
        ++dark_count;
    }

    g_line_dark_count = dark_count;

    if ((position == NULL) || (darkness_sum < minimum_darkness))
    {
        return 0U;
    }

    *position = GRAY_ARRAY_SIGN * (weighted_sum / darkness_sum);
    return 1U;
}

void Gray_SetCalibration(const uint16_t *white, const uint16_t *black)
{
    Grayscale_SetCalibration(white, black);
    memcpy(gray_calibration_white, gray_white, sizeof(gray_calibration_white));
    memcpy(gray_calibration_black, gray_black, sizeof(gray_calibration_black));
}

void Gray_Init(void)
{
    Grayscale_Init();
    memcpy(gray_calibration_white, gray_white, sizeof(gray_calibration_white));
    memcpy(gray_calibration_black, gray_black, sizeof(gray_calibration_black));
    gray_calibration_stage = GRAY_CALIBRATION_WAIT_WHITE;
    gray_calibrating = 0U;
    gray_line_acquire_ticks = 0U;
    gray_line_lost_ticks = 0U;
    g_line_detected = 0U;
    Gray_Task();
    Uart_Printf(DEBUG_UART,
                "Gray_Init LQ8 AS=PB1 S0/S1/S2=PC5/PC4/PD15 %s\r\n",
                (gray_online != 0U) ? "OK" : "FAIL");
}

HAL_StatusTypeDef Gray_CalibrateStep(void)
{
    HAL_StatusTypeDef status;

    gray_calibrating = 1U;
    /* Gray_Task now runs in the 10 ms control interrupt and drives the same ADC
     * and multiplexer address lines. The flag above makes it skip, but setting
     * the flag does not abort a frame already in progress. Wait out two control
     * ticks so the interrupt is provably outside Grayscale_ReadAll before the
     * calibration takes the hardware. */
    HAL_Delay(25U);
    g_line_detected = 0U;
    gray_line_acquire_ticks = 0U;
    gray_line_lost_ticks = 0U;
    TrackN = 0U;
    OLED_Clear();

    if (gray_calibration_stage == GRAY_CALIBRATION_WAIT_WHITE)
    {
        status = Gray_SampleAverage(gray_calibration_white);
        if (status == HAL_OK)
        {
            gray_calibration_stage = GRAY_CALIBRATION_WAIT_BLACK;
            Gray_PrintCalibration("white", gray_calibration_white);
            Uart_Printf(DEBUG_UART, "gray calibration next: black\r\n");
            Oled_Printf(0, 0, "GRAY CALIBRATION");
            Oled_Printf(0, 1, "WHITE: OK");
            Oled_Printf(0, 2, "NEXT: BLACK");
            Oled_Printf(0, 3, "PRESS KEY1");
        }
    }
    else
    {
        status = Gray_SampleAverage(gray_calibration_black);
        if (status == HAL_OK)
        {
            Gray_SetCalibration(gray_calibration_white, gray_calibration_black);
            gray_calibration_stage = GRAY_CALIBRATION_WAIT_WHITE;
            Gray_PrintCalibration("black", gray_calibration_black);
            Uart_Printf(DEBUG_UART,
                        "gray calibration applied (RAM only), next: white\r\n");
            Oled_Printf(0, 0, "GRAY CALIBRATION");
            Oled_Printf(0, 1, "BLACK: OK");
            Oled_Printf(0, 2, "RAM APPLIED");
            Oled_Printf(0, 3, "NEXT: WHITE");
        }
    }

    if (status != HAL_OK)
    {
        Uart_Printf(DEBUG_UART, "gray calibration capture failed\r\n");
        Oled_Printf(0, 0, "GRAY CALIBRATION");
        Oled_Printf(0, 1, "CAPTURE FAILED");
        Oled_Printf(0, 2, "RETRY: %s",
                    (gray_calibration_stage == GRAY_CALIBRATION_WAIT_WHITE) ?
                    "WHITE" : "BLACK");
        Oled_Printf(0, 3, "PRESS KEY1");
    }

    HAL_Delay(GRAY_CALIBRATION_HOLD_MS);
    gray_calibrating = 0U;
    return status;
}

GrayCalibrationStage_t Gray_GetCalibrationStage(void)
{
    return gray_calibration_stage;
}

uint8_t Gray_Is_Calibrating(void)
{
    return gray_calibrating;
}

void Gray_Task(void)
{
    float raw_position;
    uint8_t position_valid;

    if (gray_calibrating != 0U)
    {
        return;
    }

    if (Grayscale_ReadAll() != HAL_OK)
    {
        gray_online = 0U;
        g_line_detected = 0U;
        gray_line_acquire_ticks = 0U;
        gray_line_lost_ticks = 0U;
        return;
    }

    memcpy(Anolog, gray_raw, sizeof(Anolog));
    memcpy(Normal, gray_normalized, sizeof(Normal));
    TrackN = (uint8_t)((uint8_t)~Grayscale_GetWhiteMask() &
                       Grayscale_GetValidMask());

    gray_online = 1U;
    gray_last_update_tick = HAL_GetTick();

    position_valid = Gray_CalculatePosition(&raw_position);
    if ((position_valid != 0U) &&
        !(raw_position >= -GRAY_POSITION_LIMIT &&
          raw_position <= GRAY_POSITION_LIMIT))
    {
        position_valid = 0U;
    }

    /* TrackN supplies the per-channel Schmitt hysteresis, while the analog
     * centroid remains the steering input. Confirm acquisition for two frames
     * and tolerate two bad frames so threshold noise cannot toggle the line
     * state every control tick. During that grace period the last filtered
     * position is intentionally held. */
    if ((position_valid != 0U) && (TrackN != 0U))
    {
        gray_line_lost_ticks = 0U;
        if (!(g_line_position_error >= -GRAY_POSITION_LIMIT &&
              g_line_position_error <= GRAY_POSITION_LIMIT))
        {
            g_line_position_error = raw_position;
        }
        else
        {
            g_line_position_error +=
                GRAY_LPF_ALPHA * (raw_position - g_line_position_error);
        }

        if (g_line_detected == 0U)
        {
            if (gray_line_acquire_ticks < GRAY_LINE_ACQUIRE_TICKS)
            {
                ++gray_line_acquire_ticks;
            }
            if (gray_line_acquire_ticks >= GRAY_LINE_ACQUIRE_TICKS)
            {
                g_line_detected = 1U;
            }
        }
        else
        {
            gray_line_acquire_ticks = GRAY_LINE_ACQUIRE_TICKS;
        }
        return;
    }

    gray_line_acquire_ticks = 0U;
    if (g_line_detected == 0U)
    {
        gray_line_lost_ticks = 0U;
        return;
    }

    if (gray_line_lost_ticks < GRAY_LINE_LOST_TICKS)
    {
        ++gray_line_lost_ticks;
    }
    if (gray_line_lost_ticks >= GRAY_LINE_LOST_TICKS)
    {
        g_line_detected = 0U;
    }
}

uint8_t Gray_Is_Online(void)
{
    return gray_online;
}

uint8_t Gray_Is_Data_Fresh(uint32_t max_age_ms)
{
    return ((gray_calibrating == 0U) &&
            (gray_online != 0U) &&
            ((HAL_GetTick() - gray_last_update_tick) <= max_age_ms)) ? 1U : 0U;
}

uint32_t Gray_Get_Last_Update_Tick(void)
{
    return gray_last_update_tick;
}
