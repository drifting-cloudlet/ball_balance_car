#ifndef __LED_APP_H__
#define __LED_APP_H__

#include "MyDefine.h"

void Led_Init(void);
void Led_Task(void);

extern uint8_t led_buf[4];

#endif
