#include "key_driver.h"

uint8_t Key_Read(void)
{
    uint8_t key = 0U;

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        key = USER_BUTTON_1;
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        key = USER_BUTTON_2;
    }
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
    {
        key = USER_BUTTON_3;
    }
    if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET)
    {
        key = USER_BUTTON_4;
    }
    return key;
}

uint8_t Key_ReadMissionButtons(void)
{
    uint8_t pressed = 0U;

    if (HAL_GPIO_ReadPin(Q3_START_GPIO_Port, Q3_START_Pin) == GPIO_PIN_RESET)
    {
        pressed |= Q3_BUTTON_START_MASK;
    }
    if (HAL_GPIO_ReadPin(Q3_CENTER_GPIO_Port, Q3_CENTER_Pin) == GPIO_PIN_RESET)
    {
        /* PC1 keeps its legacy CubeMX label; its application action is Q6 HOLD. */
        pressed |= Q6_BUTTON_HOLD_MASK;
    }
    if (HAL_GPIO_ReadPin(Q4_START_GPIO_Port, Q4_START_Pin) == GPIO_PIN_RESET)
    {
        pressed |= Q4_BUTTON_START_MASK;
    }
    if (HAL_GPIO_ReadPin(Q5_START_GPIO_Port, Q5_START_Pin) == GPIO_PIN_RESET)
    {
        pressed |= Q5_BUTTON_START_MASK;
    }
    if (HAL_GPIO_ReadPin(Q6_TOGGLE_GPIO_Port, Q6_TOGGLE_Pin) == GPIO_PIN_RESET)
    {
        pressed |= Q6_BUTTON_TOGGLE_MASK;
    }

    return pressed;
}
