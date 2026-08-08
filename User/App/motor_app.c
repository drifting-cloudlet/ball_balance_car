#include "motor_app.h"

MOTOR left_motor;
MOTOR right_motor;

void Motor_Init(void)
{
    Uart_Printf(DEBUG_UART, "Motor_Init ......\r\n");
  
    /* Dead band is zero here on purpose. Compensating for it inside the driver
     * lifted every output in (0, 550) up to 550 of a 999 count period, so the
     * command jumped straight from -55% to +55% whenever the loop crossed zero.
     * The compensation now lives in the speed loop as a feedforward term, where
     * it stays continuous. See SPEED_FF_DUTY_PERCENT in pid_app.h. */
    Motor_Config_Init(&left_motor, &htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_1, 0, 0);
    Motor_Config_Init(&right_motor, &htim1, TIM_CHANNEL_4, &htim1, TIM_CHANNEL_3, 1, 0);
}

void Motor_Task(void)
{

}

void Motor_Stop_All(void)
{
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
}

void Motor_Brake_All(void)
{
    Motor_Brake(&left_motor);
    Motor_Brake(&right_motor);
}
