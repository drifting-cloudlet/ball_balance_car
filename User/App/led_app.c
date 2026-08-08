#include "led_app.h"

uint8_t led_buf[4] = {0, 0, 0, 0};

void Led_Init()
{
    Uart_Printf(DEBUG_UART, "Led_Init ......\r\n");
    Led_Display(led_buf, 0);
}

void Led_Task()
{
    Led_Display(led_buf, 1);
}
