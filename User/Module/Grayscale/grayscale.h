#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "adc.h"
#include "gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAY_CHANNEL_COUNT        8U
#define GRAY_ADC_FULL_SCALE       4095U
#define GRAY_DEFAULT_WHITE_VALUE  1700U
#define GRAY_DEFAULT_BLACK_VALUE  400U

extern uint16_t gray_raw[GRAY_CHANNEL_COUNT];
extern uint16_t gray_normalized[GRAY_CHANNEL_COUNT];
extern uint16_t gray_white[GRAY_CHANNEL_COUNT];
extern uint16_t gray_black[GRAY_CHANNEL_COUNT];

void Grayscale_Init(void);
void Grayscale_SetCalibration(const uint16_t *white, const uint16_t *black);
HAL_StatusTypeDef Grayscale_ReadAll(void);
uint8_t Grayscale_GetWhiteMask(void);
uint8_t Grayscale_GetValidMask(void);

#ifdef __cplusplus
}
#endif

#endif
