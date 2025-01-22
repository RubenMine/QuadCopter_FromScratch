#include "header/pid.h"

void init_pid_controllers() {
    pid_init(&roll_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&pitch_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&yaw_pid, 1.0f, 0.0f, 0.0f);
    pid_init(&altitude_pid, 1.0f, 0.0f, 0.0f);
    Serial.println("[DEBUG] PIDs Inizializzati!");

}

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
