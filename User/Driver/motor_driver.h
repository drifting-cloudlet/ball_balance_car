#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "main.h"

// DRV8871DDA ������ýṹ��
typedef struct MOTOR_Config
{
    struct 
    {
        TIM_HandleTypeDef *htim; // ��ʱ��
        uint32_t pwm_channel; // PWM ͨ��
    } in1, in2; // ��Ӧ DRV8871DDA �е� AIN1 �� AIN2
    unsigned char reverse; // ����ķ����Ƿ�ת��0-������1-��ת
} MOTOR_Config;


// ����ṹ��
typedef struct MOTOR
{
    MOTOR_Config config;
    int speed;
    int dead_band_speed; // �������� -dead_band_speed < speed < dead_band_speed ʱ�������ת����
} MOTOR;



// �����ʼ������(����ص�PWMͨ��)
void Motor_Config_Init(MOTOR* motor, 
                       TIM_HandleTypeDef *htim_in1, uint32_t pwm_channel_in1,
                       TIM_HandleTypeDef *htim_in2, uint32_t pwm_channel_in2,
                       unsigned char reverse, int dead_band_speed);

// �ٶȿ���
void Motor_Set_Speed(MOTOR* motor, int speed);

/* Duty percent, -100.0 to +100.0. Preferred over Motor_Set_Speed for closed
 * loops: no dead-band lift, so the command stays continuous through zero. */
void Motor_Set_Duty(MOTOR* motor, float percent);

// ���ֹͣ  
void Motor_Stop(MOTOR* motor);          

// ���ɲ��
void Motor_Brake(MOTOR* motor);         

#endif
