#ifndef PID_H
#define PID_H

#include <Arduino.h>

typedef struct PID {
    int Kp, Ki, Kd;
    float Integrator_STATE, Filter_STATE;
    float setpoint, output;
    float UpperLimit, LowerLimit;
    bool anti_windup;
} PID;

// Dichiarazione dei PID globali (da definire nel modulo di inizializzazione)
extern PID roll_pid, pitch_pid, yaw_pid, altitude_pid;

void pid_init(PID* pid, int kp, int ki, int kd, float UpperLimit, float LowerLimit, float setpoint, bool anti_windup);
void pid_compute(PID* pid, float sensor_value);
void drone_control();

#endif
