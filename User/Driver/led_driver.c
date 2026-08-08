#include "led_driver.h"

void Led_Display(uint8_t *led_buf, uint8_t enable)
{
  uint8_t led_temp = 0x00;
  static uint8_t led_temp_old = 0x00;
  
  if(enable)
    led_temp = (led_buf[3] << 3) | (led_buf[2] << 2) | (led_buf[1] << 1) | led_buf[0];
  
  if(led_temp != led_temp_old)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led_buf[0] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led_buf[1] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, led_buf[2] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, led_buf[3] ? GPIO_PIN_RESET : GPIO_PIN_SET);

    led_temp_old = led_temp;
  }
}
