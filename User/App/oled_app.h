#ifndef __OLED_APP_H__
#define __OLED_APP_H__

#include "MyDefine.h"

/* Four-row ready/calibration dashboard. Runtime display I/O is suppressed until
 * the selected mission finishes, then Oled_RuntimeTask writes the final mode,
 * state, and time once. */
void Oled_Init(void);
void Oled_Task(void);
void Oled_RuntimeTask(void);

#endif
