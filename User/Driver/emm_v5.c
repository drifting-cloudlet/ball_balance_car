#include "emm_v5.h"
#include <string.h>

#define EMM_UART_TIMEOUT_MS 20U
#define EMM_UART4_TX_QUEUE_DEPTH 8U
#define EMM_TX_MAX_FRAME_SIZE    20U

typedef struct
{
    uint8_t data[EMM_TX_MAX_FRAME_SIZE];
    uint8_t length;
} EmmTxFrame_t;

typedef struct
{
    UART_HandleTypeDef *uart;
    EmmTxFrame_t ordered[EMM_UART4_TX_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t inflight[EMM_TX_MAX_FRAME_SIZE];
    uint8_t latest_position[EMM_TX_MAX_FRAME_SIZE];
    uint8_t latest_position_length;
    uint8_t latest_position_valid;
    volatile uint8_t busy;
} EmmUart4Tx_t;

static EmmUart4Tx_t emm_uart4_tx;

static uint32_t Emm_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Emm_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint8_t Emm_IsUart4(const UART_HandleTypeDef *uart)
{
    return ((uart != NULL) && (uart->Instance == UART4)) ? 1U : 0U;
}

/* Called with interrupts masked. Ordered control frames always run before the
 * coalesced position slot; both sources are copied into stable storage before
 * HAL starts consuming them from the UART interrupt. */
static void Emm_Uart4KickLocked(void)
{
    const EmmTxFrame_t *ordered_frame = NULL;
    const uint8_t *source;
    uint8_t length;

    if ((emm_uart4_tx.busy != 0U) || (emm_uart4_tx.uart == NULL))
    {
        return;
    }

    if (emm_uart4_tx.count != 0U)
    {
        ordered_frame = &emm_uart4_tx.ordered[emm_uart4_tx.head];
        source = ordered_frame->data;
        length = ordered_frame->length;
    }
    else if (emm_uart4_tx.latest_position_valid != 0U)
    {
        source = emm_uart4_tx.latest_position;
        length = emm_uart4_tx.latest_position_length;
    }
    else
    {
        return;
    }

    memcpy(emm_uart4_tx.inflight, source, length);
    if (HAL_UART_Transmit_IT(emm_uart4_tx.uart,
                             emm_uart4_tx.inflight, length) != HAL_OK)
    {
        /* Keep the frame pending. The next command or completion callback will
         * retry without blocking the control loop. */
        return;
    }

    emm_uart4_tx.busy = 1U;
    if (ordered_frame != NULL)
    {
        emm_uart4_tx.head = (uint8_t)((emm_uart4_tx.head + 1U) %
                                      EMM_UART4_TX_QUEUE_DEPTH);
        --emm_uart4_tx.count;
    }
    else
    {
        emm_uart4_tx.latest_position_valid = 0U;
    }
}

static void Emm_Uart4QueueOrdered(UART_HandleTypeDef *uart,
                                  const uint8_t *command, uint8_t length)
{
    uint32_t primask;
    EmmTxFrame_t *frame;

    if ((length == 0U) || (length > EMM_TX_MAX_FRAME_SIZE))
    {
        return;
    }

    primask = Emm_EnterCritical();
    emm_uart4_tx.uart = uart;

    /* Even if an abnormal command burst fills the ordered queue, never let a
     * stale position target run after the newer control intent. */
    emm_uart4_tx.latest_position_valid = 0U;
    if (emm_uart4_tx.count < EMM_UART4_TX_QUEUE_DEPTH)
    {
        frame = &emm_uart4_tx.ordered[emm_uart4_tx.tail];
        memcpy(frame->data, command, length);
        frame->length = length;
        emm_uart4_tx.tail = (uint8_t)((emm_uart4_tx.tail + 1U) %
                                      EMM_UART4_TX_QUEUE_DEPTH);
        ++emm_uart4_tx.count;
    }
    Emm_Uart4KickLocked();
    Emm_ExitCritical(primask);
}

static void Emm_Uart4QueueLatestPosition(UART_HandleTypeDef *uart,
                                         const uint8_t *command, uint8_t length)
{
    uint32_t primask;

    if ((length == 0U) || (length > EMM_TX_MAX_FRAME_SIZE))
    {
        return;
    }

    primask = Emm_EnterCritical();
    emm_uart4_tx.uart = uart;
    memcpy(emm_uart4_tx.latest_position, command, length);
    emm_uart4_tx.latest_position_length = length;
    emm_uart4_tx.latest_position_valid = 1U;
    Emm_Uart4KickLocked();
    Emm_ExitCritical(primask);
}

static void Emm_Send(UART_HandleTypeDef *uart, const uint8_t *command,
                     uint8_t length)
{
    if ((uart == NULL) || (command == NULL) || (length == 0U))
    {
        return;
    }

    if (Emm_IsUart4(uart) != 0U)
    {
        Emm_Uart4QueueOrdered(uart, command, length);
        return;
    }

    (void)HAL_UART_Transmit(uart, (uint8_t *)command, length,
                            EMM_UART_TIMEOUT_MS);
}

void Emm_V5_TxInit(void)
{
    uint32_t primask = Emm_EnterCritical();

    memset(&emm_uart4_tx, 0, sizeof(emm_uart4_tx));
    Emm_ExitCritical(primask);
}

bool Emm_V5_TxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t primask;

    if (Emm_IsUart4(uart) == 0U)
    {
        return false;
    }

    primask = Emm_EnterCritical();
    emm_uart4_tx.uart = uart;
    emm_uart4_tx.busy = 0U;
    Emm_Uart4KickLocked();
    Emm_ExitCritical(primask);
    return true;
}

void Emm_V5_Reset_CurPos_To_Zero(UART_HandleTypeDef *uart, uint8_t addr)
{
    uint8_t command[] = {addr, 0x0AU, 0x6DU, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Reset_Clog_Pro(UART_HandleTypeDef *uart, uint8_t addr)
{
    uint8_t command[] = {addr, 0x0EU, 0x52U, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Read_Sys_Params(UART_HandleTypeDef *uart, uint8_t addr, SysParams_t parameter)
{
    uint8_t command[4];
    uint8_t length = 1U;

    command[0] = addr;
    switch (parameter)
    {
        case S_VER:   command[length++] = 0x1FU; break;
        case S_RL:    command[length++] = 0x20U; break;
        case S_PID:   command[length++] = 0x21U; break;
        case S_VBUS:  command[length++] = 0x24U; break;
        case S_CPHA:  command[length++] = 0x27U; break;
        case S_ENCL:  command[length++] = 0x31U; break;
        case S_TPOS:  command[length++] = 0x33U; break;
        case S_VEL:   command[length++] = 0x35U; break;
        case S_CPOS:  command[length++] = 0x36U; break;
        case S_PERR:  command[length++] = 0x37U; break;
        case S_FLAG:  command[length++] = 0x3AU; break;
        case S_ORG:   command[length++] = 0x3BU; break;
        case S_Conf:  command[length++] = 0x42U; command[length++] = 0x6CU; break;
        case S_State: command[length++] = 0x43U; command[length++] = 0x7AU; break;
        default: return;
    }
    command[length++] = 0x6BU;
    Emm_Send(uart, command, length);
}

void Emm_V5_Modify_Ctrl_Mode(UART_HandleTypeDef *uart, uint8_t addr, bool save,
                             uint8_t control_mode)
{
    uint8_t command[] = {addr, 0x46U, 0x69U, (uint8_t)save, control_mode, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_En_Control(UART_HandleTypeDef *uart, uint8_t addr, bool state, bool sync)
{
    uint8_t command[] = {addr, 0xF3U, 0xABU, (uint8_t)state, (uint8_t)sync, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Vel_Control(UART_HandleTypeDef *uart, uint8_t addr, uint8_t direction,
                        uint16_t velocity, uint8_t acceleration, bool sync)
{
    uint8_t command[] = {
        addr, 0xF6U, direction,
        (uint8_t)(velocity >> 8), (uint8_t)velocity,
        acceleration, (uint8_t)sync, 0x6BU,
    };
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Pos_Control(UART_HandleTypeDef *uart, uint8_t addr, uint8_t direction,
                        uint16_t velocity, uint8_t acceleration, uint32_t pulses,
                        bool absolute, bool sync)
{
    uint8_t command[] = {
        addr, 0xFDU, direction,
        (uint8_t)(velocity >> 8), (uint8_t)velocity, acceleration,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses,
        (uint8_t)absolute, (uint8_t)sync, 0x6BU,
    };
    if (Emm_IsUart4(uart) != 0U)
    {
        Emm_Uart4QueueLatestPosition(uart, command, sizeof(command));
    }
    else
    {
        Emm_Send(uart, command, sizeof(command));
    }
}

void Emm_V5_Stop_Now(UART_HandleTypeDef *uart, uint8_t addr, bool sync)
{
    uint8_t command[] = {addr, 0xFEU, 0x98U, (uint8_t)sync, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Synchronous_Motion(UART_HandleTypeDef *uart, uint8_t addr)
{
    uint8_t command[] = {addr, 0xFFU, 0x66U, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Origin_Set_Zero(UART_HandleTypeDef *uart, uint8_t addr, bool save)
{
    uint8_t command[] = {addr, 0x93U, 0x88U, (uint8_t)save, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Origin_Modify_Params(UART_HandleTypeDef *uart, uint8_t addr, bool save,
                                 uint8_t mode, uint8_t direction, uint16_t velocity,
                                 uint32_t timeout_ms, uint16_t collision_velocity,
                                 uint16_t collision_current, uint16_t collision_time_ms,
                                 bool power_on_return)
{
    uint8_t command[] = {
        addr, 0x4CU, 0xAEU, (uint8_t)save, mode, direction,
        (uint8_t)(velocity >> 8), (uint8_t)velocity,
        (uint8_t)(timeout_ms >> 24), (uint8_t)(timeout_ms >> 16),
        (uint8_t)(timeout_ms >> 8), (uint8_t)timeout_ms,
        (uint8_t)(collision_velocity >> 8), (uint8_t)collision_velocity,
        (uint8_t)(collision_current >> 8), (uint8_t)collision_current,
        (uint8_t)(collision_time_ms >> 8), (uint8_t)collision_time_ms,
        (uint8_t)power_on_return, 0x6BU,
    };
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Origin_Trigger_Return(UART_HandleTypeDef *uart, uint8_t addr,
                                  uint8_t mode, bool sync)
{
    uint8_t command[] = {addr, 0x9AU, mode, (uint8_t)sync, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}

void Emm_V5_Origin_Interrupt(UART_HandleTypeDef *uart, uint8_t addr)
{
    uint8_t command[] = {addr, 0x9CU, 0x48U, 0x6BU};
    Emm_Send(uart, command, sizeof(command));
}
