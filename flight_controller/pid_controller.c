#include <stdio.h>
#include <stdint.h>
#include "sensors.c"

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral, previous_error;
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float pid_compute(PIDController *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
