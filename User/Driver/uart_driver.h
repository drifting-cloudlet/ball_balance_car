#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ringbuffer.h"

/* Bring-up with a single USB-TTL adapter: the Zigbee module on UART5 is the
 * only link to the laptop, so console output has to ride it too. Set back to 0
 * as soon as a second adapter frees up USART1.
 *
 * UART5 runs at 9600, which is 960 bytes per second. The transmit queue is
 * asynchronous so nothing blocks, but a burst of printing will overflow the
 * 1 KB ring buffer and lines will go missing without warning. Keep the output
 * sparse while this is 1. */
#define DEBUG_ON_BRINGUP_LINK 1

#if DEBUG_ON_BRINGUP_LINK
#define DEBUG_UART &huart5
#else
#define DEBUG_UART &huart1
#endif

#define BUFFER_SIZE 128 // ��������С

typedef struct
{
    uint32_t events;
    uint32_t bytes;
    uint32_t dropped;
    uint32_t errors;
    uint32_t recoveries;
    uint32_t start_failures;
    uint32_t last_error;
} Uart6RxDiagnostics_t;

void Uart_Tx_Init(void);

HAL_StatusTypeDef Uart6_RxStart(void);
void Uart6_RxEnsureStarted(void);
void Uart6_RxGetDiagnostics(Uart6RxDiagnostics_t *diagnostics);
void Uart6_RxResetDiagnostics(void);

int Uart_Printf(UART_HandleTypeDef *huart, const char *format, ...);  

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


#endif
