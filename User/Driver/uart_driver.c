#include "uart_driver.h"
#include "emm_v5.h"

// UART发送队列相关定义
#define UART_TX_BUFFER_SIZE   1024U  // 环形缓冲区大小
#define UART_TX_INFLIGHT_SIZE 128U   // 单次启动 HAL_UART_Transmit_IT 的最大字节数

static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];   // 发送环形缓冲区底层数组
static struct rt_ringbuffer uart_tx_ringbuffer;       // 发送环形缓冲区
static uint8_t uart_tx_inflight[UART_TX_INFLIGHT_SIZE]; // 正在异步发送的数据副本
static volatile uint8_t uart_tx_busy = 0;             // 0-空闲，1-忙

/* 临界区：保存并屏蔽中断，再恢复。ISR 与主循环都可调用。 */
static inline uint32_t Uart_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void Uart_Exit_Critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* 假设已在临界区内：尝试启动一次发送 */
static void Uart_Tx_Kick(UART_HandleTypeDef *huart)
{
    rt_size_t data_len;

    if (uart_tx_busy != 0U)
    {
        return;
    }

    data_len = rt_ringbuffer_get(&uart_tx_ringbuffer,
                                 uart_tx_inflight,
                                 (rt_uint16_t)UART_TX_INFLIGHT_SIZE);
    if (data_len == 0U)
    {
        return;
    }

    uart_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(huart, uart_tx_inflight, (uint16_t)data_len) != HAL_OK)
    {
        uart_tx_busy = 0U;  /* 启动失败，让下一次调用重试 */
    }
}

void Uart_Tx_Init(void)
{
    rt_ringbuffer_init(&uart_tx_ringbuffer, uart_tx_buffer, (rt_int16_t)UART_TX_BUFFER_SIZE);
    uart_tx_busy = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (Emm_V5_TxCpltCallback(huart))
    {
        return;
    }

    if (huart != DEBUG_UART)
    {
        return;
    }

    primask = Uart_Enter_Critical();
    uart_tx_busy = 0U;
    Uart_Tx_Kick(huart);
    Uart_Exit_Critical(primask);
}

int Uart_Printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    char temp_buffer[512];
    va_list arg;
    int len;

    va_start(arg, format);
    len = vsnprintf(temp_buffer, sizeof(temp_buffer), format, arg);
    va_end(arg);

    if (len <= 0)
    {
        return 0;
    }

    /* vsnprintf returns the size it wanted, not the number stored. Never let
     * that value make the ring buffer read beyond temp_buffer. */
    if (len >= (int)sizeof(temp_buffer))
    {
        len = (int)sizeof(temp_buffer) - 1;
    }

    if (huart == DEBUG_UART)
    {
        uint32_t primask = Uart_Enter_Critical();
        if (rt_ringbuffer_space_len(&uart_tx_ringbuffer) >= (rt_size_t)len)
        {
            (void)rt_ringbuffer_put(&uart_tx_ringbuffer,
                                    (uint8_t *)temp_buffer,
                                    (rt_uint16_t)len);
        }
        else
        {
            len = 0;  /* drop the whole line; never send a misleading fragment */
        }
        Uart_Tx_Kick(huart);
        Uart_Exit_Critical(primask);
    }
    else
    {
        HAL_UART_Transmit(huart, (uint8_t *)temp_buffer, (uint16_t)len, 0xFF);
    }

    return len;
}

/* 串口 1 */
uint8_t uart1_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart1_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart1_ring_buffer; // 环形缓冲区

uint8_t uart1_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 2 */
uint8_t uart2_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart2_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart2_ring_buffer; // 环形缓冲区

uint8_t uart2_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 4 */
uint8_t uart4_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart4_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart4_ring_buffer; // 环形缓冲区

uint8_t uart4_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 5 */
uint8_t uart5_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart5_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart5_ring_buffer; // 环形缓冲区

uint8_t uart5_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 6 */
uint8_t uart6_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart6_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart6_ring_buffer; // 环形缓冲区

uint8_t uart6_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

static volatile Uart6RxDiagnostics_t uart6_rx_diagnostics;

static HAL_StatusTypeDef Uart6_StartRxInternal(void)
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart6, uart6_rx_dma_buffer, sizeof(uart6_rx_dma_buffer));

    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
    else
    {
        ++uart6_rx_diagnostics.start_failures;
    }
    return status;
}

HAL_StatusTypeDef Uart6_RxStart(void)
{
    return Uart6_StartRxInternal();
}

void Uart6_RxEnsureStarted(void)
{
    if (huart6.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }

    (void)HAL_UART_DMAStop(&huart6);
    if (Uart6_StartRxInternal() == HAL_OK)
    {
        ++uart6_rx_diagnostics.recoveries;
    }
}

void Uart6_RxGetDiagnostics(Uart6RxDiagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL)
    {
        return;
    }

    primask = Uart_Enter_Critical();
    diagnostics->events = uart6_rx_diagnostics.events;
    diagnostics->bytes = uart6_rx_diagnostics.bytes;
    diagnostics->dropped = uart6_rx_diagnostics.dropped;
    diagnostics->errors = uart6_rx_diagnostics.errors;
    diagnostics->recoveries = uart6_rx_diagnostics.recoveries;
    diagnostics->start_failures = uart6_rx_diagnostics.start_failures;
    diagnostics->last_error = uart6_rx_diagnostics.last_error;
    Uart_Exit_Critical(primask);
}

void Uart6_RxResetDiagnostics(void)
{
    uint32_t primask = Uart_Enter_Critical();

    memset((void *)&uart6_rx_diagnostics, 0, sizeof(uart6_rx_diagnostics));
    Uart_Exit_Critical(primask);
}

/* 串口空闲中断 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* 串口 1 */
    if (huart->Instance == USART1)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart1_ring_buffer, uart1_rx_dma_buffer, Size);

        memset(uart1_rx_dma_buffer, 0, sizeof(uart1_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    
    /* 串口 2 */
    if (huart->Instance == USART2)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart2_ring_buffer, uart2_rx_dma_buffer, Size);

        memset(uart2_rx_dma_buffer, 0, sizeof(uart2_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, sizeof(uart2_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    
    /* 串口 4 */
    if (huart->Instance == UART4)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart4_ring_buffer, uart4_rx_dma_buffer, Size);

        memset(uart4_rx_dma_buffer, 0, sizeof(uart4_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
    
    /* 串口 5 */
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart5_ring_buffer, uart5_rx_dma_buffer, Size);

        memset(uart5_rx_dma_buffer, 0, sizeof(uart5_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, sizeof(uart5_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    
    /* 串口 6 */
    if (huart->Instance == USART6)
    {
        rt_size_t written;

        HAL_UART_DMAStop(huart);

        written = rt_ringbuffer_put(&uart6_ring_buffer,
                                    uart6_rx_dma_buffer, Size);
        ++uart6_rx_diagnostics.events;
        uart6_rx_diagnostics.bytes += Size;
        uart6_rx_diagnostics.dropped += (uint32_t)Size - (uint32_t)written;

        memset(uart6_rx_dma_buffer, 0, sizeof(uart6_rx_dma_buffer));

        (void)Uart6_StartRxInternal();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        ++uart6_rx_diagnostics.errors;
        uart6_rx_diagnostics.last_error = huart->ErrorCode;

        if (Uart6_StartRxInternal() == HAL_OK)
        {
            ++uart6_rx_diagnostics.recoveries;
        }
        return;
    }

    /* Keep the UART5/9600 bench console usable after noise or an overrun. The
     * HAL has already ended the failed DMA receive before calling us. */
    if (huart->Instance == UART5)
    {
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer,
                                        sizeof(uart5_rx_dma_buffer)) == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
        }
    }
}
