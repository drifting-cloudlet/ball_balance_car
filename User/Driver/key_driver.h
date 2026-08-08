#ifndef __KEY_DRIVER_H__
#define __KEY_DRIVER_H__

#include "main.h"

typedef enum
{
    USER_BUTTON_1 = 1,
    USER_BUTTON_2 = 2,
    USER_BUTTON_3 = 3,
    USER_BUTTON_4 = 4,
} user_button_t;

#define Q3_BUTTON_START_MASK  (1U << 0)
#define Q6_BUTTON_HOLD_MASK   (1U << 1)
#define Q4_BUTTON_START_MASK  (1U << 2)
#define Q5_BUTTON_START_MASK  (1U << 3)
#define Q6_BUTTON_TOGGLE_MASK (1U << 4)

uint8_t Key_Read(void);
uint8_t Key_ReadMissionButtons(void);

#endif
