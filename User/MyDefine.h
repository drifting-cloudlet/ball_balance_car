#ifndef __MYDEFINE_H__
#define __MYDEFINE_H__

/* ========== HAL ��ͷ�ļ� ========== */
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* ========== C ����ͷ�ļ� ========== */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* ========== �����ͷ�ļ� ========== */
#include "ringbuffer.h"

#include "oled.h"

#include "icm20608.h"

#include "grayscale.h"

#include "madgwickFilter.h"

#include "pid.h"

/* ========== ������ͷ�ļ� ========== */
#include "led_driver.h"
#include "key_driver.h"
#include "uart_driver.h"
#include "oled_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "gyroscope_driver.h"
#include "emm_v5.h"

/* ========== Ӧ�ò�ͷ�ļ� ========== */
#include "led_app.h"
#include "key_app.h"
#include "uart_app.h"
#include "oled_app.h"
#include "motor_app.h"
#include "encoder_app.h"
#include "gyroscope_app.h"
#include "pid_app.h"
#include "gray_app.h"
#include "step_app.h"
#include "pendulum_app.h"
#include "ball_app.h"
#include "balance_app.h"
#include "mission_app.h"
/* ========== ���ĵ�����ͷ�ļ� ========== */
#include "Scheduler.h"
#include "Scheduler_Task.h"

#endif
