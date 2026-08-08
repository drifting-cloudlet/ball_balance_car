#ifndef __EMM_V5_H__
#define __EMM_V5_H__

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    S_VER = 0,
    S_RL = 1,
    S_PID = 2,
    S_VBUS = 3,
    S_CPHA = 5,
    S_ENCL = 7,
    S_TPOS = 8,
    S_VEL = 9,
    S_CPOS = 10,
    S_PERR = 11,
    S_FLAG = 13,
    S_Conf = 14,
    S_State = 15,
    S_ORG = 16,
} SysParams_t;

/* UART4 uses an interrupt-driven ordered queue plus one coalesced position
 * slot. Other Emm UARTs retain the existing blocking transport. */
void Emm_V5_TxInit(void);
bool Emm_V5_TxCpltCallback(UART_HandleTypeDef *uart);

void Emm_V5_Reset_CurPos_To_Zero(UART_HandleTypeDef *uart, uint8_t addr);
void Emm_V5_Reset_Clog_Pro(UART_HandleTypeDef *uart, uint8_t addr);
void Emm_V5_Read_Sys_Params(UART_HandleTypeDef *uart, uint8_t addr, SysParams_t parameter);
void Emm_V5_Modify_Ctrl_Mode(UART_HandleTypeDef *uart, uint8_t addr, bool save,
                             uint8_t control_mode);
void Emm_V5_En_Control(UART_HandleTypeDef *uart, uint8_t addr, bool state, bool sync);
void Emm_V5_Vel_Control(UART_HandleTypeDef *uart, uint8_t addr, uint8_t direction,
                        uint16_t velocity, uint8_t acceleration, bool sync);
void Emm_V5_Pos_Control(UART_HandleTypeDef *uart, uint8_t addr, uint8_t direction,
                        uint16_t velocity, uint8_t acceleration, uint32_t pulses,
                        bool absolute, bool sync);
void Emm_V5_Stop_Now(UART_HandleTypeDef *uart, uint8_t addr, bool sync);
void Emm_V5_Synchronous_Motion(UART_HandleTypeDef *uart, uint8_t addr);
void Emm_V5_Origin_Set_Zero(UART_HandleTypeDef *uart, uint8_t addr, bool save);
void Emm_V5_Origin_Modify_Params(UART_HandleTypeDef *uart, uint8_t addr, bool save,
                                 uint8_t mode, uint8_t direction, uint16_t velocity,
                                 uint32_t timeout_ms, uint16_t collision_velocity,
                                 uint16_t collision_current, uint16_t collision_time_ms,
                                 bool power_on_return);
void Emm_V5_Origin_Trigger_Return(UART_HandleTypeDef *uart, uint8_t addr,
                                  uint8_t mode, bool sync);
void Emm_V5_Origin_Interrupt(UART_HandleTypeDef *uart, uint8_t addr);

#endif
