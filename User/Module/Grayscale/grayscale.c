#include "grayscale.h"

/* LQ 8LIGHT: AS=PB1/ADC1_IN9, S0=PC5, S1=PC4, S2=PD15. */
#define GRAY_S0_GPIO_Port GPIOC
#define GRAY_S0_Pin       GPIO_PIN_5
#define GRAY_S1_GPIO_Port GPIOC
#define GRAY_S1_Pin       GPIO_PIN_4
#define GRAY_S2_GPIO_Port GPIOD
#define GRAY_S2_Pin       GPIO_PIN_15

#define GRAY_DISCARD_SAMPLE_COUNT 3U
#define GRAY_AVERAGE_SAMPLE_COUNT 2U
#define GRAY_TOTAL_SAMPLE_COUNT   \
    (GRAY_DISCARD_SAMPLE_COUNT + GRAY_AVERAGE_SAMPLE_COUNT)
#define GRAY_ADC_TIMEOUT_MS       1U

uint16_t gray_raw[GRAY_CHANNEL_COUNT] = {0U};
uint16_t gray_normalized[GRAY_CHANNEL_COUNT] = {0U};
uint16_t gray_white[GRAY_CHANNEL_COUNT] = {0U};
uint16_t gray_black[GRAY_CHANNEL_COUNT] = {0U};

static uint16_t gray_white_threshold[GRAY_CHANNEL_COUNT] = {0U};
static uint16_t gray_black_threshold[GRAY_CHANNEL_COUNT] = {0U};
static uint8_t gray_white_mask = 0U;
static uint8_t gray_valid_mask = 0U;
static uint8_t gray_white_is_high_mask = 0U;

static void Grayscale_SelectChannel(uint8_t channel)
{
    HAL_GPIO_WritePin(GRAY_S0_GPIO_Port,
                      GRAY_S0_Pin,
                      ((channel & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_S1_GPIO_Port,
                      GRAY_S1_Pin,
                      ((channel & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_S2_GPIO_Port,
                      GRAY_S2_Pin,
                      ((channel & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Grayscale_Settle(void)
{
    volatile uint32_t delay;

    for (delay = 0U; delay < 200U; ++delay)
    {
        __NOP();
    }
}

static HAL_StatusTypeDef Grayscale_ReadAdc(uint16_t *value)
{
    HAL_StatusTypeDef status;

    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_PollForConversion(&hadc1, GRAY_ADC_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        *value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    (void)HAL_ADC_Stop(&hadc1);
    return status;
}

static uint16_t Grayscale_Normalize(uint8_t channel, uint16_t value)
{
    uint32_t normalized;

    if ((gray_valid_mask & (uint8_t)(1U << channel)) == 0U)
    {
        return 0U;
    }

    if ((gray_white_is_high_mask & (uint8_t)(1U << channel)) != 0U)
    {
        if (value <= gray_black[channel])
        {
            return 0U;
        }

        if (value >= gray_white[channel])
        {
            return GRAY_ADC_FULL_SCALE;
        }

        normalized = ((uint32_t)(value - gray_black[channel]) * GRAY_ADC_FULL_SCALE) /
                     (uint32_t)(gray_white[channel] - gray_black[channel]);
    }
    else
    {
        if (value >= gray_black[channel])
        {
            return 0U;
        }

        if (value <= gray_white[channel])
        {
            return GRAY_ADC_FULL_SCALE;
        }

        normalized = ((uint32_t)(gray_black[channel] - value) * GRAY_ADC_FULL_SCALE) /
                     (uint32_t)(gray_black[channel] - gray_white[channel]);
    }

    return (uint16_t)normalized;
}

void Grayscale_SetCalibration(const uint16_t *white, const uint16_t *black)
{
    uint8_t channel;

    gray_white_mask = 0U;
    gray_valid_mask = 0U;
    gray_white_is_high_mask = 0U;

    for (channel = 0U; channel < GRAY_CHANNEL_COUNT; ++channel)
    {
        uint16_t white_value = (white != NULL) ? white[channel] : GRAY_DEFAULT_WHITE_VALUE;
        uint16_t black_value = (black != NULL) ? black[channel] : GRAY_DEFAULT_BLACK_VALUE;

        if (white_value > GRAY_ADC_FULL_SCALE)
        {
            white_value = GRAY_ADC_FULL_SCALE;
        }
        if (black_value > GRAY_ADC_FULL_SCALE)
        {
            black_value = GRAY_ADC_FULL_SCALE;
        }

        gray_white[channel] = white_value;
        gray_black[channel] = black_value;

        if (white_value != black_value)
        {
            if (white_value > black_value)
            {
                gray_white_is_high_mask |= (uint8_t)(1U << channel);
            }

            gray_white_threshold[channel] =
                (uint16_t)(((uint32_t)white_value * 2U + black_value) / 3U);
            gray_black_threshold[channel] =
                (uint16_t)(((uint32_t)white_value + (uint32_t)black_value * 2U) / 3U);
            gray_valid_mask |= (uint8_t)(1U << channel);
        }
        else
        {
            gray_white_threshold[channel] = white_value;
            gray_black_threshold[channel] = black_value;
        }
    }
}

void Grayscale_Init(void)
{
    uint8_t channel;

    gray_white_mask = 0U;
    for (channel = 0U; channel < GRAY_CHANNEL_COUNT; ++channel)
    {
        gray_raw[channel] = 0U;
        gray_normalized[channel] = 0U;
    }

    Grayscale_SetCalibration(NULL, NULL);
    Grayscale_SelectChannel(0U);
}

HAL_StatusTypeDef Grayscale_ReadAll(void)
{
    uint16_t frame[GRAY_CHANNEL_COUNT];
    uint8_t mux_channel;

    for (mux_channel = 0U; mux_channel < GRAY_CHANNEL_COUNT; ++mux_channel)
    {
        uint32_t sum = 0U;
        uint8_t sample_index;

        Grayscale_SelectChannel(mux_channel);
        Grayscale_Settle();

        for (sample_index = 0U; sample_index < GRAY_TOTAL_SAMPLE_COUNT; ++sample_index)
        {
            uint16_t sample;

            if (Grayscale_ReadAdc(&sample) != HAL_OK)
            {
                return HAL_ERROR;
            }
            if (sample_index >= GRAY_DISCARD_SAMPLE_COUNT)
            {
                sum += sample;
            }
        }

        frame[mux_channel] = (uint16_t)(sum / GRAY_AVERAGE_SAMPLE_COUNT);
    }

    for (mux_channel = 0U; mux_channel < GRAY_CHANNEL_COUNT; ++mux_channel)
    {
        uint8_t channel_bit = (uint8_t)(1U << mux_channel);
        uint8_t white_is_high =
            (uint8_t)(gray_white_is_high_mask & channel_bit);

        gray_raw[mux_channel] = frame[mux_channel];
        gray_normalized[mux_channel] = Grayscale_Normalize(mux_channel, frame[mux_channel]);

        if (((white_is_high != 0U) &&
             (frame[mux_channel] > gray_white_threshold[mux_channel])) ||
            ((white_is_high == 0U) &&
             (frame[mux_channel] < gray_white_threshold[mux_channel])))
        {
            gray_white_mask |= channel_bit;
        }
        else if (((white_is_high != 0U) &&
                  (frame[mux_channel] < gray_black_threshold[mux_channel])) ||
                 ((white_is_high == 0U) &&
                  (frame[mux_channel] > gray_black_threshold[mux_channel])))
        {
            gray_white_mask &= (uint8_t)~channel_bit;
        }
    }

    return HAL_OK;
}

uint8_t Grayscale_GetWhiteMask(void)
{
    return gray_white_mask;
}

uint8_t Grayscale_GetValidMask(void)
{
    return gray_valid_mask;
}
