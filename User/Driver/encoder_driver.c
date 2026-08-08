#include "encoder_driver.h"

/**
 * @brief 初始化编码器驱动
 */
void Encoder_Driver_Init(Encoder* encoder, TIM_HandleTypeDef *htim, unsigned char reverse)
{
  encoder->htim = htim;
  encoder->reverse = reverse;

  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
  __HAL_TIM_SetCounter(encoder->htim, 0);

  encoder->count = 0;
  encoder->total_count = 0;
  encoder->speed_cm_s = 0.0f;
  encoder->rpm = 0.0f;
}

/**
 * @brief 更新编码器数据
 * @note 建议每 10ms 调用一次
 */
void Encoder_Driver_Update(Encoder* encoder)
{
  encoder->count = (int16_t)__HAL_TIM_GetCounter(encoder->htim);
  encoder->count = (encoder->reverse == 0) ? encoder->count : -encoder->count;

  __HAL_TIM_SetCounter(encoder->htim, 0);

  encoder->total_count += encoder->count;

  encoder->speed_cm_s = (float)encoder->count / ENCODER_PPR * WHEEL_CIRCUMFERENCE_CM / SAMPLING_TIME_S;
  encoder->rpm = (float)encoder->count / ENCODER_PPR * (60.0f / SAMPLING_TIME_S);
}
