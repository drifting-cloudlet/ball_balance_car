#ifndef __GRAY_APP_H__
#define __GRAY_APP_H__

#include "grayscale.h"
#include <stdint.h>

typedef enum
{
    GRAY_CALIBRATION_WAIT_WHITE = 0U,
    GRAY_CALIBRATION_WAIT_BLACK = 1U,
} GrayCalibrationStage_t;

void Gray_Init(void);
void Gray_Task(void);
void Gray_SetCalibration(const uint16_t *white, const uint16_t *black);
HAL_StatusTypeDef Gray_CalibrateStep(void);
GrayCalibrationStage_t Gray_GetCalibrationStage(void);
uint8_t Gray_Is_Calibrating(void);
uint8_t Gray_Is_Online(void);
uint8_t Gray_Is_Data_Fresh(uint32_t max_age_ms);
uint32_t Gray_Get_Last_Update_Tick(void);

extern uint16_t Anolog[GRAY_CHANNEL_COUNT];
extern uint16_t Normal[GRAY_CHANNEL_COUNT];
extern uint8_t TrackN;
extern volatile float g_line_position_error;
extern volatile uint8_t g_line_detected;
extern volatile uint8_t g_line_dark_count;

#endif
