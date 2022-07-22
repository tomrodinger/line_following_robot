#ifndef MOTOR_H
#define MOTOR_H

typedef enum {
    FORWARD = 0,
    BACKWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    CIRCLE_LEFT,
    CIRCLE_RIGHT,
    STOP
} motor_direction_t;

void motor_init(void);
void motor_run(motor_direction_t dir, uint8_t speed);
void motor_run_manual(int8_t r_speed, int8_t l_speed);

#endif