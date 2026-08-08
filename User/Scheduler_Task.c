#include "Scheduler_Task.h"

/* Bring-up switches live next to the code they affect, which means nothing
 * stops a build going to the track with one still flipped. Evaluate them all in
 * one place and make the firmware say so out loud at boot. */
#if DEBUG_ON_BRINGUP_LINK || \
    (BALL_SOURCE == BALL_SOURCE_UART5) || \
    PENDULUM_PROBE_MODE || \
    SPEED_PID_DEBUG_MODE || \
    (!PENDULUM_ENERGISE_AT_BOOT) || \
    (!PENDULUM_LINKAGE_ATTACHED)
#define SYSTEM_BRINGUP_BUILD 1
#else
#define SYSTEM_BRINGUP_BUILD 0
#endif

static void System_PrintBuildFlags(void)
{
#if SYSTEM_BRINGUP_BUILD
  Uart_Printf(DEBUG_UART, "==== BRING-UP BUILD, NOT COMPETITION READY ====\r\n");
#if SPEED_PID_DEBUG_MODE
  Uart_Printf(DEBUG_UART,
              "  LINE FOLLOWING COMPILED OUT, KEY2 runs both wheels at %d cm/s\r\n",
              SPEED_PID_DEBUG_TARGET_CM_S);
#endif
#if DEBUG_ON_BRINGUP_LINK
  Uart_Printf(DEBUG_UART, "  console on UART5 9600, USART1 idle\r\n");
#endif
#if BALL_SOURCE == BALL_SOURCE_UART5
  Uart_Printf(DEBUG_UART, "  ball frames on UART5, not the production USART6\r\n");
#endif
#if PENDULUM_PROBE_MODE
  Uart_Printf(DEBUG_UART, "  keys remapped to ZDT probes\r\n");
#endif
#if !PENDULUM_ENERGISE_AT_BOOT
  Uart_Printf(DEBUG_UART, "  pendulum released at boot, \"step:1\" to zero and hold\r\n");
#endif
#if !PENDULUM_LINKAGE_ATTACHED
  Uart_Printf(DEBUG_UART, "  no linkage declared, probes use wide travel\r\n");
#endif
#else
  Uart_Printf(DEBUG_UART, "==== competition build ====\r\n");
#endif
}

/* Control-tick timing, measured with the DWT cycle counter.
 *
 * Moving the grayscale read into the control interrupt is only safe if it
 * actually fits there, and 64 blocking ADC conversions is not an obviously
 * small number. The reference project instruments its control tick for the same
 * reason: a number measured on the hardware beats an estimate from a datasheet. */
static volatile uint32_t gray_cycles = 0U;
static volatile uint32_t gray_cycles_max = 0U;
static volatile uint32_t ctrl_cycles = 0U;
static volatile uint32_t ctrl_cycles_max = 0U;

static void System_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void System_Init(void)
{
  System_EnableCycleCounter();
  Uart_Tx_Init();
  Uart_Printf(DEBUG_UART, "==== System Init ====\r\n");
  Led_Init();
  Key_Init();
  Uart_Init();
  Oled_Init();
  Motor_Init();
  Encoder_Init();
  Gyroscope_Init();
  PID_Init();
  Gray_Init();
  Emm_V5_TxInit();
  Step_Init();
  Pendulum_Init();
  Ball_Init();
  Balance_Init();
  Mission_Init();
  System_PrintBuildFlags();
  Uart_Printf(DEBUG_UART, "==== Finish ====\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
}

static uint8_t timer10ms = 0U;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t start;

  if (htim->Instance != htim2.Instance)
  {
    return;
  }

  if (++timer10ms < 10U)
  {
    return;
  }
  timer10ms = 0U;

  start = DWT->CYCCNT;

  /* Order matters and must not be rearranged.
   *
   * Gray_Task first, in the same tick as the controller that uses its output.
   * The line PID differentiates g_line_position_error, and a derivative is only
   * meaningful if the signal it differentiates advances once per tick. Sampling
   * it from the main loop instead put the two on separate clocks. */
  Gray_Task();
  gray_cycles = DWT->CYCCNT - start;

  Encoder_Task();
  /* Between the encoder and the PID: it reads the distance just updated and
   * writes the speed the PID is about to act on. */
  Mission_Task();
  PID_Speed_Task();

  ctrl_cycles = DWT->CYCCNT - start;
  if (ctrl_cycles > ctrl_cycles_max)
  {
    ctrl_cycles_max = ctrl_cycles;
  }
  if (gray_cycles > gray_cycles_max)
  {
    gray_cycles_max = gray_cycles;
  }
}

void System_TickStats(uint32_t *gray_us, uint32_t *gray_us_max,
                      uint32_t *ctrl_us, uint32_t *ctrl_us_max)
{
  const uint32_t per_us = SystemCoreClock / 1000000U;

  if (gray_us != NULL)     { *gray_us = gray_cycles / per_us; }
  if (gray_us_max != NULL) { *gray_us_max = gray_cycles_max / per_us; }
  if (ctrl_us != NULL)     { *ctrl_us = ctrl_cycles / per_us; }
  if (ctrl_us_max != NULL) { *ctrl_us_max = ctrl_cycles_max / per_us; }
}

void System_TickStatsReset(void)
{
  gray_cycles_max = 0U;
  ctrl_cycles_max = 0U;
}
