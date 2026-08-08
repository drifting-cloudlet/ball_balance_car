#ifndef __UART_APP_H__
#define __UART_APP_H__

#include "MyDefine.h"

void Uart_Init(void);

/* Parse one console command line. Exposed so a port that carries both typed
 * commands and machine frames can dispatch on the frame header and hand the
 * leftovers here. */
void Uart_Parse_Command(char *command);

void Uart1_Task(void);
void Uart2_Task(void);
void Uart4_Task(void);
void Uart5_Task(void);
void Uart6_Task(void);

#endif
