#include "Scheduler.h"

typedef struct
{
  void (*task_func)(void);
  uint32_t rate_ms;
  uint32_t last_run;
} scheduler_task_t;

#define SCHEDULER_TASK_COUNT(tasks) \
  ((uint8_t)(sizeof(tasks) / sizeof((tasks)[0])))

uint8_t task_num;

/* The ball loop does not consume the IMU. With the default zero line-gyro
 * damping, reading it during ball balancing only spends about 1.5 ms in a
 * blocking 100 kHz I2C transfer. Keep it available if gyro damping is enabled
 * for a later combined driving-and-balancing run. */
static void Scheduler_GyroscopeBackgroundTask(void)
{
  if ((Balance_IsEnabled() == 0U) || (line_gyro_kd != 0.0f))
  {
    Gyroscope_Task();
  }
}

/* Oled_Task writes every glyph byte with a separate blocking I2C transaction.
 * Show the full dashboard only while selecting a mission. Every running state
 * is display-silent; Oled_RuntimeTask writes the final mode/state and time once
 * at completion. */
static void Scheduler_OledBackgroundTask(void)
{
  if (Mission_GetState() == MISSION_IDLE)
  {
    Oled_Task();
    return;
  }

  Oled_RuntimeTask();
}

/* These tasks form one control transaction. Whenever they are due they all run
 * before any background work, in producer-to-consumer order. Pendulum_Task
 * queues UART4 work from the main loop; the ISR only advances prepared frames. */
static scheduler_task_t control_tasks[] =
{
  {Ball_Task, BALL_TASK_PERIOD_MS, 0U},
  {Balance_Task, BALANCE_TASK_PERIOD_MS, 0U},
  {Pendulum_Task, PEND_TASK_PERIOD_MS, 0U},
};

/* At most one background task runs per main-loop pass. A long task cannot sit
 * between Ball_Task, Balance_Task and Pendulum_Task when they become due on the
 * same tick. Gray_Task remains in the 10 ms TIM2 control callback. */
static scheduler_task_t background_tasks[] =
{
  {Key_Task, 5U, 0U},
#if DEBUG_ON_BRINGUP_LINK
  {Uart5_Task, 20U, 0U},
#else
  {Uart1_Task, 20U, 0U},
#endif
  {PID_MonitorTask, 20U, 0U},
  {Scheduler_GyroscopeBackgroundTask, 10U, 0U},
  {Scheduler_OledBackgroundTask, 100U, 0U},
};

void Scheduler_Init(void)
{
  task_num = SCHEDULER_TASK_COUNT(control_tasks) +
             SCHEDULER_TASK_COUNT(background_tasks);
}

void Scheduler_Run(void)
{
  uint32_t now_time = HAL_GetTick();
  uint8_t index;

  for (index = 0U; index < SCHEDULER_TASK_COUNT(control_tasks); ++index)
  {
    if ((now_time - control_tasks[index].last_run) >=
        control_tasks[index].rate_ms)
    {
      control_tasks[index].last_run = now_time;
      control_tasks[index].task_func();
    }
  }

  now_time = HAL_GetTick();
  for (index = 0U; index < SCHEDULER_TASK_COUNT(background_tasks); ++index)
  {
    if ((now_time - background_tasks[index].last_run) >=
        background_tasks[index].rate_ms)
    {
      background_tasks[index].last_run = now_time;
      background_tasks[index].task_func();
      break;
    }
  }
}
