#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

#include "main.h"

// 编码器每转一圈的脉冲数 (PPR)
#define ENCODER_PPR (13 * 28 * 4) // 13线/相, 28倍减速比, 4倍频
// 车轮直径 (单位: 厘米)
#define WHEEL_DIAMETER_CM 6.5f

// 自动计算周长和采样时间
#define ENCODER_PI 3.14159265f
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * ENCODER_PI)
#define SAMPLING_TIME_S 0.01f // 采样时间, 与定时调用的任务周期一致 (10ms)

/**
 * @brief 编码器数据结构体
 */
typedef struct
{
  TIM_HandleTypeDef *htim; // 定时器
  unsigned char reverse; // 编码器的方向是否反转。0-正常，1-反转
  int16_t count;          // 当前采样周期内的原始计数值
  int32_t total_count;    // 累计总计数值
  float speed_cm_s;     // 计算出的速度 (cm/s)
  float rpm;            // 计算出的转速 (RPM - 每分钟转数)
} Encoder;

void Encoder_Driver_Init(Encoder* encoder, TIM_HandleTypeDef *htim, unsigned char reverse);
void Encoder_Driver_Update(Encoder* encoder);

#endif
