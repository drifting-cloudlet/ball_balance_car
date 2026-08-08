#ifndef __KEY_APP_H__
#define __KEY_APP_H__

#include "MyDefine.h"

extern uint8_t key_val;
extern uint8_t key_old;
extern uint8_t key_down;
extern uint8_t key_up;

void Key_Init(void);
void Key_Task(void);

#endif
