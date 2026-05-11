#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int kp;
    int ki;
    int kd;
    int integral;
    int last_error;
    int integral_limit;
    int output_limit;
} pid_t;

void pid_init(pid_t *pid, int kp, int ki, int kd, int output_limit, int integral_limit);
void pid_reset(pid_t *pid);
int pid_update(pid_t *pid, int error);
int pid_calc(int error);

#endif
