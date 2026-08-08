#include "encoder_app.h"

// 编码器实例
Encoder left_encoder;
Encoder right_encoder;

static float Encoder_Count_To_Distance_Cm(int32_t count)
{
  if(count < 0)
  {
    count = -count;
  }

  return ((float)count / (float)ENCODER_PPR) * WHEEL_CIRCUMFERENCE_CM;
}

/**
 * @brief 初始化编码器应用
 */
void Encoder_Init(void)
{
  Uart_Printf(DEBUG_UART, "Encoder_Init ......\r\n");
  
  Encoder_Driver_Init(&left_encoder, &htim3, 0);
  Encoder_Driver_Init(&right_encoder, &htim4, 0);
}

/**
 * @brief 编码器应用运行任务 (应由调度器周期性调用)
 */
void Encoder_Task(void)
{
  Encoder_Driver_Update(&left_encoder);
  Encoder_Driver_Update(&right_encoder);
}

void Encoder_Reset_Total_Count(void)
{
  __HAL_TIM_SetCounter(left_encoder.htim, 0);
  __HAL_TIM_SetCounter(right_encoder.htim, 0);

  left_encoder.count = 0;
  left_encoder.total_count = 0;
  left_encoder.speed_cm_s = 0.0f;
  left_encoder.rpm = 0.0f;

  right_encoder.count = 0;
  right_encoder.total_count = 0;
  right_encoder.speed_cm_s = 0.0f;
  right_encoder.rpm = 0.0f;
}

float Encoder_Get_Left_Distance_Cm(void)
{
  return Encoder_Count_To_Distance_Cm(left_encoder.total_count);
}

float Encoder_Get_Right_Distance_Cm(void)
{
  return Encoder_Count_To_Distance_Cm(right_encoder.total_count);
}

float Encoder_Get_Average_Distance_Cm(void)
{
  return (Encoder_Get_Left_Distance_Cm() + Encoder_Get_Right_Distance_Cm()) * 0.5f;
}

void Encoder_Debug_Print(void)
{
  Uart_Printf(DEBUG_UART,
              "MOTOR speed L=%.2f R=%.2f cm/s rpm L=%.1f R=%.1f target=%d,%d\r\n",
              left_encoder.speed_cm_s,
              right_encoder.speed_cm_s,
              left_encoder.rpm,
              right_encoder.rpm,
              left_speed_target,
              right_speed_target);
}
