#include "uart_app.h"

void Uart_Parse_Command(char *command)
{
  float target = 0.0f;

  if(command == NULL)
  {
    return;
  }

  while((*command == ' ') || (*command == '\t'))
  {
    command++;
  }

  if(sscanf(command, "left:%f", &target) == 1)
  {
    left_speed_target = (int)target;
    Uart_Printf(DEBUG_UART, "left speed target set to %d\r\n", left_speed_target);
    return;
  }

  if(sscanf(command, "right:%f", &target) == 1)
  {
    right_speed_target = (int)target;
    Uart_Printf(DEBUG_UART, "right speed target set to %d\r\n", right_speed_target);
    return;
  }

  if(sscanf(command, "base:%f", &target) == 1)
  {
    basic_speed = (int)target;
    if(basic_speed > 0)
    {
      basic_speed = 0;
    }
    if(basic_speed < -80)
    {
      basic_speed = -80;
    }
    left_speed_target = basic_speed;
    right_speed_target = basic_speed;
    Uart_Printf(DEBUG_UART, "base speed set to %d\r\n", basic_speed);
    return;
  }

  if(sscanf(command, "yaw:%f", &target) == 1)
  {
    Uart_Printf(DEBUG_UART, "yaw cmd disabled\r\n");
    return;
  }

  if(strcmp(command, "gray") == 0)
  {
    Uart_Printf(DEBUG_UART,
                "gray raw=%u,%u,%u,%u,%u,%u,%u,%u normal=%u,%u,%u,%u,%u,%u,%u,%u mask=0x%02X line=%u err=%.1f\r\n",
                Anolog[0], Anolog[1], Anolog[2], Anolog[3],
                Anolog[4], Anolog[5], Anolog[6], Anolog[7],
                Normal[0], Normal[1], Normal[2], Normal[3],
                Normal[4], Normal[5], Normal[6], Normal[7],
                TrackN, g_line_detected, g_line_position_error);
    return;
  }

  /* Open-loop wheel drive, for measuring the real dead band. Send "dead:0"
   * first so the compensation does not silently raise whatever you ask for. */
  if(sscanf(command, "pwm:%f", &target) == 1)
  {
    PID_Set_Enable(0U);
    Motor_Set_Speed(&left_motor, (int)target);
    Motor_Set_Speed(&right_motor, (int)target);
    Uart_Printf(DEBUG_UART,
                "raw pwm %d of %lu, dead band %d\r\n",
                (int)target,
                (unsigned long)left_motor.config.in1.htim->Init.Period,
                left_motor.dead_band_speed);
    return;
  }

  if(sscanf(command, "dead:%f", &target) == 1)
  {
    left_motor.dead_band_speed = (int)target;
    right_motor.dead_band_speed = (int)target;
    Uart_Printf(DEBUG_UART, "dead band set to %d\r\n", (int)target);
    return;
  }

  if(strcmp(command, "spd") == 0)
  {
    Encoder_Debug_Print();
    return;
  }

  if(strcmp(command, "pid") == 0)
  {
    PID_PrintStatus();
    return;
  }

  if(strcmp(command, "mis") == 0)
  {
    Mission_PrintStatus();
    return;
  }

  if(strcmp(command, "run") == 0)
  {
    Mission_Start();
    return;
  }

  if(strcmp(command, "stop") == 0)
  {
    Mission_Abort();
    return;
  }

  if(sscanf(command, "mis:%f", &target) == 1)
  {
    mission_enabled = (target != 0.0f) ? 1U : 0U;
    Uart_Printf(DEBUG_UART, "mission %s, KEY2 is %s\r\n",
                (mission_enabled != 0U) ? "on" : "off",
                (mission_enabled != 0U) ? "Q2 start/abort" : "plain PID toggle");
    return;
  }

  if(sscanf(command, "xline:%f", &target) == 1)
  {
    if(target < 2.0f) target = 2.0f;
    if(target > 8.0f) target = 8.0f;
    mission_cross_min_channels = (uint8_t)target;
    Uart_Printf(DEBUG_UART, "finish line needs %u dark channels\r\n",
                (unsigned int)mission_cross_min_channels);
    return;
  }

  if(strcmp(command, "tick") == 0)
  {
    uint32_t gray_us;
    uint32_t gray_max;
    uint32_t ctrl_us;
    uint32_t ctrl_max;

    System_TickStats(&gray_us, &gray_max, &ctrl_us, &ctrl_max);
    Uart_Printf(DEBUG_UART,
                "tick gray=%luus(max %lu) ctrl=%luus(max %lu) of 10000us\r\n",
                (unsigned long)gray_us, (unsigned long)gray_max,
                (unsigned long)ctrl_us, (unsigned long)ctrl_max);
    return;
  }

  if(strcmp(command, "tickreset") == 0)
  {
    System_TickStatsReset();
    Uart_Printf(DEBUG_UART, "tick maxima cleared\r\n");
    return;
  }

  if(sscanf(command, "mon:%f", &target) == 1)
  {
    PID_SetMonitor((uint32_t)target);
    Uart_Printf(DEBUG_UART, "monitor %ums\r\n", (unsigned int)target);
    return;
  }

  if(sscanf(command, "skp:%f", &target) == 1)
  {
    pid_set_params(&pid_speed_left, target, pid_speed_left.ki, pid_speed_left.kd);
    pid_set_params(&pid_speed_right, target, pid_speed_right.ki, pid_speed_right.kd);
    Uart_Printf(DEBUG_UART, "speed kp=%.2f ki=%.2f kd=%.2f\r\n",
                target, pid_speed_left.ki, pid_speed_left.kd);
    return;
  }

  if(sscanf(command, "ski:%f", &target) == 1)
  {
    pid_set_params(&pid_speed_left, pid_speed_left.kp, target, pid_speed_left.kd);
    pid_set_params(&pid_speed_right, pid_speed_right.kp, target, pid_speed_right.kd);
    Uart_Printf(DEBUG_UART, "speed kp=%.2f ki=%.2f kd=%.2f\r\n",
                pid_speed_left.kp, target, pid_speed_left.kd);
    return;
  }

  if(sscanf(command, "skd:%f", &target) == 1)
  {
    pid_set_params(&pid_speed_left, pid_speed_left.kp, pid_speed_left.ki, target);
    pid_set_params(&pid_speed_right, pid_speed_right.kp, pid_speed_right.ki, target);
    Uart_Printf(DEBUG_UART, "speed kp=%.2f ki=%.2f kd=%.2f\r\n",
                pid_speed_left.kp, pid_speed_left.ki, target);
    return;
  }

  if(strcmp(command, "gyro") == 0)
  {
    Uart_Printf(DEBUG_UART,
                "gyro rate=%.2fdeg/s bias=%.3f calibrated=%u usable=%u kd=%.3f\r\n",
                Gyroscope_Get_Yaw_Rate(),
                euler_angles.gz_bias,
                (unsigned int)euler_angles.calibrated,
                (unsigned int)Gyroscope_Rate_Is_Usable(LINE_GYRO_MAX_AGE_MS),
                line_gyro_kd);
    return;
  }

  if(sscanf(command, "gkd:%f", &target) == 1)
  {
    if(target < -1.0f) target = -1.0f;
    if(target >  1.0f) target =  1.0f;
    line_gyro_kd = target;
    Uart_Printf(DEBUG_UART, "gyro damping kd=%.3f\r\n", line_gyro_kd);
    return;
  }

  if(sscanf(command, "ff:%f", &target) == 1)
  {
    if(target < 0.0f)  target = 0.0f;
    if(target > 60.0f) target = 60.0f;
    speed_ff_duty = target;
    Uart_Printf(DEBUG_UART, "speed feedforward %.1f%% duty\r\n", speed_ff_duty);
    return;
  }

  /* Requirement select. Named after the requirement number because that is what
   * is written on the bench, the scoring sheet and the report. */
  if(strcmp(command, "m2") == 0)
  {
    Mission_SetMode(MISSION_MODE_LAP);
    return;
  }

  /* Hold mode. "hold" arms it at the target already set, "hold:<mm>" sets the
   * target too. This is the step before m3 and the basis of Q4/Q5/Q6. */
  if(sscanf(command, "hold:%f", &target) == 1)
  {
    Mission_SetHoldTarget(target * 0.001f);
    Mission_SetMode(MISSION_MODE_HOLD);
    return;
  }

  if(strcmp(command, "hold") == 0)
  {
    Mission_SetMode(MISSION_MODE_HOLD);
    return;
  }

  if(strcmp(command, "m3") == 0)
  {
    Mission_SetMode(MISSION_MODE_BALL);
    return;
  }

  if(strcmp(command, "m4") == 0)
  {
    Mission_SetMode(MISSION_MODE_Q4);
    return;
  }

  if(strcmp(command, "m5") == 0)
  {
    Mission_SetMode(MISSION_MODE_Q5);
    return;
  }

  if(strcmp(command, "ball") == 0)
  {
    Ball_PrintStats();
    return;
  }

  if(strcmp(command, "bal") == 0)
  {
    Balance_PrintStatus();
    return;
  }

  /* Clear peak error and saturation count. Engage, let the transient pass, then
   * "balrst" to read the steady-state peak rather than the startup swing. */
  if(strcmp(command, "balrst") == 0)
  {
    Balance_ResetInstruments();
    Uart_Printf(DEBUG_UART, "balance peak/sat cleared\r\n");
    return;
  }

  /* Mechanical direction only, not a PID gain. */
  if(sscanf(command, "msign:%f", &target) == 1)
  {
    Balance_SetMotorSign(target);
    Balance_PrintStatus();
    return;
  }

  if(sscanf(command, "bal:%f", &target) == 1)
  {
    Balance_SetEnable((target != 0.0f) ? 1U : 0U);
    return;
  }

  if(sscanf(command, "balx:%f", &target) == 1)
  {
    Mission_SetHoldTarget(target * 0.001f);   /* command is in mm */
    Uart_Printf(DEBUG_UART, "balance/Q6 target %.1fmm\r\n",
                Mission_GetHoldTarget() * 1000.0f);
    return;
  }

  if(strcmp(command, "zero") == 0)
  {
    Pendulum_ZeroHere();
    return;
  }

  if(sscanf(command, "step:%f", &target) == 1)
  {
    Pendulum_SetEnergised((target != 0.0f) ? 1U : 0U);
    Uart_Printf(DEBUG_UART, "pendulum %s\r\n",
                (Pendulum_IsEnergised() != 0U) ? "ENERGISED" : "released");
    return;
  }

  if(strcmp(command, "motor") == 0)
  {
    Uart_Printf(DEBUG_UART,
                "motor target=%.2f applied=%.2f deg\r\n",
                Pendulum_GetTargetMotorDeg(),
                Pendulum_GetAppliedMotorDeg());
    return;
  }

  if(sscanf(command, "motor:%f", &target) == 1)
  {
    if(Balance_IsEnabled() != 0U)
    {
      Uart_Printf(DEBUG_UART, "refused: balance loop owns the motor\r\n");
      return;
    }
    Pendulum_SetMotorAngle(target);
    Uart_Printf(DEBUG_UART, "motor target %.2fdeg\r\n",
                Pendulum_GetTargetMotorDeg());
    return;
  }

  if(strcmp(command, "ballreset") == 0)
  {
    Ball_ResetStats();
    Uart_Printf(DEBUG_UART, "ball stats cleared\r\n");
    return;
  }

  Uart_Printf(DEBUG_UART, "unknown cmd: %s\r\n", command);
}

/* 串口 1 */
extern uint8_t uart1_rx_dma_buffer[BUFFER_SIZE];
extern uint8_t uart1_ring_buffer_input[BUFFER_SIZE];
extern struct rt_ringbuffer uart1_ring_buffer;
extern uint8_t uart1_data_buffer[BUFFER_SIZE];
/* 串口 2 */
extern uint8_t uart2_rx_dma_buffer[BUFFER_SIZE];
extern uint8_t uart2_ring_buffer_input[BUFFER_SIZE];
extern struct rt_ringbuffer uart2_ring_buffer;
extern uint8_t uart2_data_buffer[BUFFER_SIZE];
/* 串口 4 */
extern uint8_t uart4_rx_dma_buffer[BUFFER_SIZE];
extern uint8_t uart4_ring_buffer_input[BUFFER_SIZE];
extern struct rt_ringbuffer uart4_ring_buffer;
extern uint8_t uart4_data_buffer[BUFFER_SIZE];
/* 串口 5 */
extern uint8_t uart5_rx_dma_buffer[BUFFER_SIZE];
extern uint8_t uart5_ring_buffer_input[BUFFER_SIZE];
extern struct rt_ringbuffer uart5_ring_buffer;
extern uint8_t uart5_data_buffer[BUFFER_SIZE];
/* 串口 6 */
extern uint8_t uart6_rx_dma_buffer[BUFFER_SIZE];
extern uint8_t uart6_ring_buffer_input[BUFFER_SIZE];
extern struct rt_ringbuffer uart6_ring_buffer;
extern uint8_t uart6_data_buffer[BUFFER_SIZE];

void Uart_Init(void)
{
  Uart_Printf(DEBUG_UART, "Uart_Init ......\r\n");

  /* 串口 1 */
  rt_ringbuffer_init(&uart1_ring_buffer, uart1_ring_buffer_input, BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* 串口 2 */
  rt_ringbuffer_init(&uart2_ring_buffer, uart2_ring_buffer_input, BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  /* 串口 4 */
  rt_ringbuffer_init(&uart4_ring_buffer, uart4_ring_buffer_input, BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

  /* 串口 5 */
  rt_ringbuffer_init(&uart5_ring_buffer, uart5_ring_buffer_input, BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

  /* 串口 6 */
  rt_ringbuffer_init(&uart6_ring_buffer, uart6_ring_buffer_input, BUFFER_SIZE);
  if(Uart6_RxStart() != HAL_OK)
  {
    Uart_Printf(DEBUG_UART, "ERROR USART6 ball RX did not start\r\n");
  }
}

/* A DMA idle event can split one typed command anywhere. Keep a line per
 * console UART and only parse after CR/LF, instead of treating each DMA chunk
 * as a complete command. */
#define UART_COMMAND_LINE_SIZE 96U

typedef struct
{
  char data[UART_COMMAND_LINE_SIZE];
  uint16_t length;
  uint8_t discard;
} UartCommandLine_t;

static UartCommandLine_t uart1_command_line;
static UartCommandLine_t uart5_command_line;

static void Uart_CommandTask(struct rt_ringbuffer *ring,
                             UartCommandLine_t *line)
{
  rt_uint8_t byte;

  while(rt_ringbuffer_getchar(ring, &byte) != 0U)
  {
    if((byte == '\r') || (byte == '\n'))
    {
      if((line->discard == 0U) && (line->length != 0U))
      {
        line->data[line->length] = '\0';
        Uart_Parse_Command(line->data);
      }
      line->length = 0U;
      line->discard = 0U;
      continue;
    }

    if((byte == 0x08U) || (byte == 0x7FU))
    {
      if(line->length != 0U)
      {
        --line->length;
      }
      continue;
    }

    if(line->discard != 0U)
    {
      continue;
    }

    if(line->length >= (UART_COMMAND_LINE_SIZE - 1U))
    {
      line->length = 0U;
      line->discard = 1U;
      continue;
    }

    line->data[line->length++] = (char)byte;
  }
}

/* 串口 1 */
void Uart1_Task(void)
{
  Uart_CommandTask(&uart1_ring_buffer, &uart1_command_line);
}

/* 串口 2 */
void Uart2_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart2_ring_buffer);
  if(uart_data_len > 0)
  {
    if(uart_data_len >= BUFFER_SIZE) uart_data_len = BUFFER_SIZE - 1U;
    rt_ringbuffer_get(&uart2_ring_buffer, uart2_data_buffer, uart_data_len);
    uart2_data_buffer[uart_data_len] = '\0';
    Uart_Printf(DEBUG_UART, "UART2 Ringbuffer:%s\r\n", uart2_data_buffer);

    memset(uart2_data_buffer, 0, uart_data_len);
  }
}

/* 串口 4 */
void Uart4_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart4_ring_buffer);
  if(uart_data_len > 0)
  {
    if(uart_data_len >= BUFFER_SIZE) uart_data_len = BUFFER_SIZE - 1U;
    rt_ringbuffer_get(&uart4_ring_buffer, uart4_data_buffer, uart_data_len);
    uart4_data_buffer[uart_data_len] = '\0';
    Uart_Printf(DEBUG_UART, "UART4 Ringbuffer:%s\r\n", uart4_data_buffer);

    memset(uart4_data_buffer, 0, uart_data_len);
  }
}

/* 串口 5 */
void Uart5_Task(void)
{
  Uart_CommandTask(&uart5_ring_buffer, &uart5_command_line);
}

/* 串口 6 */
void Uart6_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart6_ring_buffer);
  if(uart_data_len > 0)
  {
    if(uart_data_len >= BUFFER_SIZE) uart_data_len = BUFFER_SIZE - 1U;
    rt_ringbuffer_get(&uart6_ring_buffer, uart6_data_buffer, uart_data_len);
    uart6_data_buffer[uart_data_len] = '\0';
    Uart_Printf(DEBUG_UART, "UART6 Ringbuffer:%s\r\n", uart6_data_buffer);

    memset(uart6_data_buffer, 0, uart_data_len);
  }
}
