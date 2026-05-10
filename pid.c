#include "pid.h"

#define PID_SCALE 100

static int clamp_int(int value, int min_value, int max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void pid_init(pid_t *pid, int kp, int ki, int kd, int output_limit, int integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->last_error = 0;
}

int pid_update(pid_t *pid, int error)
{
    int derivative;
    int output;

    pid->integral += error;
    pid->integral = clamp_int(pid->integral, -pid->integral_limit, pid->integral_limit);

    derivative = error - pid->last_error;
    pid->last_error = error;

    output = (pid->kp * error + pid->ki * pid->integral + pid->kd * derivative) / PID_SCALE;
    return clamp_int(output, -pid->output_limit, pid->output_limit);
}

int pid_calc(int error)
{
    static pid_t line_pid;
    static int initialized = 0;

    if (!initialized)
    {
        pid_init(&line_pid, 35, 0, 18, 45, 1000);
        initialized = 1;
    }

    return pid_update(&line_pid, error);
}
