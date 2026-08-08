#ifndef __STEP_APP_H__
#define __STEP_APP_H__

#include "emm_v5.h"

#define STEP_A_UART          (&huart4) /* STEP1: PA0 TX, PA1 RX */
#define STEP_A_ADDR          0x01U
#define STEP_B_UART          (&huart2) /* STEP2: PA2 TX, PA3 RX */
#define STEP_B_ADDR          0x01U
#define STEP_MAX_RPM         300
#define STEP_PROTOCOL_MAX_RPM 5000U
#define STEP_ACCELERATION    10U
#define STEP_SYNC            false
#define STEP_SUBDIVISION     16U
#define STEP_PULSES_PER_REV  (200UL * STEP_SUBDIVISION)

void Step_Init(void);
void Step_SetSpeed(int8_t a_percent, int8_t b_percent);
void Step_SetRPM(int16_t a_rpm, int16_t b_rpm);
void Step_Stop(void);
void Step_Disable(void);
void Step_EnableOne(UART_HandleTypeDef *uart, uint8_t addr, bool enable);
void Step_StopOne(UART_HandleTypeDef *uart, uint8_t addr);
void Step_MoveAngle(UART_HandleTypeDef *uart, uint8_t addr, float degrees,
                    uint16_t rpm, uint8_t acceleration, bool absolute);
void Step_SetZeroHere(UART_HandleTypeDef *uart, uint8_t addr);
void Step_Home(UART_HandleTypeDef *uart, uint8_t addr, uint8_t mode);

#endif
