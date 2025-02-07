// Motors.h
#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Servo.h>

// Costanti per PWM e potenza
#define PWM_MIN 1120
#define PWM_MAX 1200
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define POWER_MIN 0
#define POWER_MAX 1175

// Prototipi delle funzioni
void mixing(float h_norm, float a_roll_norm, float a_pitch_norm, float a_yaw_norm, float motor_outputs[4]);
int power_to_pwm(float power);
void compute_motor_pwm(float pid_height, float pid_roll, float pid_pitch, float pid_yaw);
void motor_power();

// Dichiarazione degli oggetti Servo (definiti in Motors.cpp)
extern Servo motor1, motor2, motor3, motor4;

// Struttura per i PWM attivi dei motori
typedef struct {
    int pw_min;
    int pw_max;
    int pwm_min;
    int pwm_max;
    int motor_pwm[4];
} power;

extern power pw;

#endif
