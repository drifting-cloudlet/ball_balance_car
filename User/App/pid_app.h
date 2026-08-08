#ifndef __PID_APP_H__
#define __PID_APP_H__

#include "MyDefine.h"

/* 1: tune both wheel-speed loops with a fixed target; 0: normal line follow. */
#define SPEED_PID_DEBUG_MODE        0
#define SPEED_PID_DEBUG_TARGET_CM_S (-45)

#if ((SPEED_PID_DEBUG_MODE != 0) && (SPEED_PID_DEBUG_MODE != 1))
#error "SPEED_PID_DEBUG_MODE must be 0 or 1"
#endif

/* Duty percent added in the direction of the speed target, to cover the duty
 * the motor needs before it turns at all. This replaces the dead band that used
 * to live inside motor_driver: applied here it is continuous, applied there it
 * quantised every command to at least +-55%.
 *
 * TODO measure. With the wheels off the ground:
 *     pwm:0            stop
 *     pwm:60, pwm:80, ... until the wheel just starts turning
 *     spd              read the encoder
 * The duty at which it first moves, as a percentage of the 999 count period, is
 * this value. Runtime tunable with "ff:<percent>" while tuning. */
#define SPEED_FF_DUTY_PERCENT       15.0f

/* Soft start: the common-mode speed target ramps instead of stepping.
 *
 * The step is not a comfort problem, it is the single largest disturbance the
 * ball ever sees. Going 0 -> 45 cm/s in 0.5 s is 0.9 m/s^2, and the tilt needed
 * to hold the ball against that is atan(0.9/9.81) = 5.2 degrees, larger than the
 * entire +-3 degree control authority. The ball leaves before the rod can
 * answer, and no gain fixes it.
 *
 * A constant acceleration is fine on its own: the rod simply holds a constant
 * tilt and the ball stays put. What hurts is the jerk at the edges, so bound the
 * acceleration and the feedforward can track it.
 *
 * 30 cm/s^2 needs 1.75 degrees of tilt, comfortably inside the limit, and takes
 * 1.5 s to reach 45 cm/s. Cost to requirement 2: about 0.8 s out of a 20 s
 * budget that currently finishes in 15.4 s. */
#define SPEED_RAMP_CM_S2            30.0f
#define SPEED_CONTROL_DT_S          0.01f
#define SPEED_RAMP_PER_TICK         (SPEED_RAMP_CM_S2 * SPEED_CONTROL_DT_S)

/* The 10 ms encoder estimate changes in 1.40 cm/s steps. Feeding those steps
 * directly into Kp/Kd makes the duty alternate by about ten percentage points
 * even at constant speed. Filter only the PID feedback; odometry and stop
 * detection continue to use the unfiltered encoder measurements. */
#define SPEED_FEEDBACK_LPF_ALPHA     0.25f

/* During the low-speed ends of a ramp, reserve at least half of the common
 * target for each wheel. Full +/-10 cm/s line authority is available once the
 * common speed reaches 20 cm/s. */
#define LINE_RAMP_STEERING_FRACTION  0.50f

/* Gyro damping on the steering command, in cm/s of differential per deg/s of
 * turn rate.
 *
 * With the sensor array leading the axle by p, the line error obeys
 *
 *     de/dt = v * psi + p * omega
 *
 * so the turn rate is one component of the error's derivative, measured
 * directly instead of differenced out of a signal quantised by the 1.2 cm
 * sensor pitch. That is the whole appeal: kd on the line error amplifies
 * quantisation steps, kd on the gyro does not.
 *
 * Rate only. An integrated heading needs a perfect bias or it ramps away; the
 * same bias in a rate is a constant offset that the stationary boot calibration
 * removes.
 *
 * Default 0 so the term is opt-in. Raise it with "gkd:<value>" while watching
 * the weave, and settle the sign first: if a small value makes the weave worse
 * rather than better, flip LINE_GYRO_SIGN. Do not tune this before the
 * grayscale-in-the-control-tick change has been verified on the car, or it will
 * mask whatever that fixed. */
#define LINE_GYRO_KD                0.0f
#define LINE_GYRO_SIGN              (+1.0f)
#define LINE_GYRO_MAX_AGE_MS        30U

typedef struct
{
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
} PidParams_t;

void PID_Init(void);
void PID_Set_Enable(uint8_t enable);
void PID_Speed_Task(void);
void PID_Task(void);
void PID_PrintStatus(void);
void PID_ResetCounters(void);

/* Periodic speed-loop telemetry. Period 0 disables it. */
void PID_SetMonitor(uint32_t period_ms);
void PID_MonitorTask(void);

extern volatile uint32_t pid_monitor_period_ms;

extern volatile uint32_t pid_drive_ticks;
extern volatile uint32_t pid_brake_ticks;

extern volatile uint8_t pid_running;
extern volatile float speed_ff_duty;
extern volatile float line_gyro_kd;
extern volatile int basic_speed;

/* Ramped common-mode speed command. Forward remains negative in the chassis
 * speed convention. */
extern volatile float speed_cmd_cm_s;

/* Signed common-mode command acceleration. Forward is negative on this
 * chassis, matching speed_cmd_cm_s. The ball controller uses this for rod
 * acceleration feedforward without coupling the timer ISR to the stepper. */
float PID_GetCommandAccelerationCmS2(void);
extern volatile int left_speed_target;
extern volatile int right_speed_target;
extern volatile float line_speed_correction;

extern PID_T pid_speed_left;
extern PID_T pid_speed_right;
extern PID_T pid_line;

#endif
