#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid.h"

static void pid_formula_incremental(PID_T *pid);
static void pid_formula_positional(PID_T *pid);
static void pid_out_limit(PID_T *pid);

void pid_init(PID_T *pid, float kp, float ki, float kd, float target, float limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->current = 0.0f;
    pid->out = 0.0f;
    pid->limit = limit;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last2_error = 0.0f;
    pid->last_out = 0.0f;
    pid->integral = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
}

void pid_set_target(PID_T *pid, float target)
{
    pid->target = target;
}

void pid_set_params(PID_T *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_limit(PID_T *pid, float limit)
{
    pid->limit = limit;
}

void pid_reset(PID_T *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last2_error = 0.0f;
    pid->out = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
}

float pid_calculate_positional(PID_T *pid, float current)
{
    pid->current = current;
    pid_formula_positional(pid);
    pid_out_limit(pid);
    return pid->out;
}

/* Positional PID with a feedforward term added before limiting.
 *
 * The feedforward carries the bias the plant needs just to start moving, so the
 * PID only has to supply the correction on top of it. Doing this at the control
 * layer, rather than as a dead-band lift inside the motor driver, keeps the
 * output continuous through zero: a driver-side lift turns every sign change
 * into a full-scale slam between +band and -band. */
float pid_calculate_positional_ff(PID_T *pid, float current, float feedforward)
{
    pid->current = current;
    pid_formula_positional(pid);
    pid->out += feedforward;
    pid_out_limit(pid);
    return pid->out;
}

float pid_calculate_incremental(PID_T *pid, float current)
{
    pid->current = current;
    pid_formula_incremental(pid);
    pid_out_limit(pid);
    return pid->out;
}

static void pid_out_limit(PID_T *pid)
{
    if (pid->out > pid->limit)
    {
        pid->out = pid->limit;
    }
    else if (pid->out < -pid->limit)
    {
        pid->out = -pid->limit;
    }
}

static void pid_formula_incremental(PID_T *pid)
{
    pid->error = pid->target - pid->current;

    pid->p_out = pid->kp * (pid->error - pid->last_error);
    pid->i_out = pid->ki * pid->error;
    pid->d_out = pid->kd * (pid->error - 2.0f * pid->last_error + pid->last2_error);

    pid->out += pid->p_out + pid->i_out + pid->d_out;

    pid->last2_error = pid->last_error;
    pid->last_error = pid->error;
}

static void pid_formula_positional(PID_T *pid)
{
    float i_max;

    pid->error = pid->target - pid->current;

    /*
     * Anti-windup, two parts, both required.
     *
     * 1) With ki == 0 the integral is forced to zero rather than left to
     *    accumulate quietly. Without this, a controller running for minutes at
     *    ki = 0 builds an enormous integral that does nothing until somebody
     *    raises ki, at which point i_out slams into the limit and stays pinned.
     *    The vehicle bolts the instant the gain is changed.
     *
     * 2) Conditional integration plus an absolute clamp. If the previous output
     *    was already at the limit and the error still pushes the same way, stop
     *    accumulating: the actuator has nothing left to give, so integrating
     *    only builds a debt that must later be paid back as overshoot. The
     *    clamp bounds |ki*integral| by the output limit, so its threshold
     *    scales with ki and needs no third magic number.
     *
     * pid->out still holds the previous tick's value here, after feedforward
     * and limiting, which is exactly what the actuator received. That is the
     * right thing to test saturation against.
     */
    if (pid->ki == 0.0f)
    {
        pid->integral = 0.0f;
    }
    else
    {
        if (!(((pid->out >= pid->limit) && (pid->error > 0.0f)) ||
              ((pid->out <= -pid->limit) && (pid->error < 0.0f))))
        {
            pid->integral += pid->error;
        }

        i_max = pid->limit / pid->ki;
        if (pid->integral > i_max)
        {
            pid->integral = i_max;
        }
        if (pid->integral < -i_max)
        {
            pid->integral = -i_max;
        }
    }

    pid->p_out = pid->kp * pid->error;
    pid->i_out = pid->ki * pid->integral;
    pid->d_out = pid->kd * (pid->error - pid->last_error);

    pid->out = pid->p_out + pid->i_out + pid->d_out;
    pid->last_error = pid->error;
}

float pid_constrain(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }

    return value;
}

void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max)
{
    if (pid->integral > max)
    {
        pid->integral = max;
    }
    else if (pid->integral < min)
    {
        pid->integral = min;
    }
}
