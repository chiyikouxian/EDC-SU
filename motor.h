#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_MAX_SPEED 100

void motor_init(void);
void motor_set_speed(int left, int right);
void motor_set_wheels(int left_front, int left_rear, int right_rear, int right_front);
void motor_set_raw(int left, int right);
void motor_update_pwm(void);
void motor_force_forward_full(void);
void motor_coast(void);
void motor_brake(void);
void motor_stop(void);
void motor_lock(void);
void motor_unlock(void);
int motor_is_locked(void);

#endif
