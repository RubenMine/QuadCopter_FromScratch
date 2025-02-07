// Motors.cpp
#include "motor.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>



Servo motor1, motor2, motor3, motor4;
power pw;


void init_esc(){
  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9);

  //sendDebug("[DEBUG] Calibrazione ESC Iniziata!");
  
  // Segnale minimo (1000 µs) per completare la calibrazione
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(4000); 

  //sendDebug("[DEBUG] Calibrazione ESC Completata!");
}

void motor_power() { 
  motor1.writeMicroseconds(pw.motor_pwm[0]);
  motor2.writeMicroseconds(pw.motor_pwm[1]);
  motor3.writeMicroseconds(pw.motor_pwm[2]);
  motor4.writeMicroseconds(pw.motor_pwm[3]);  
  
  char msg[80];
  snprintf(msg, sizeof(msg), "motor pwm: M1=%d, M2=%d, M3=%d, M4=%d", 
           pw.motor_pwm[0], pw.motor_pwm[1], pw.motor_pwm[2], pw.motor_pwm[3]);
  sendDebug(msg);
}


static float normalize(float value, float min, float max) {
    return (value - min) / (max - min);
}

static float denormalize(float value, float min, float max) {
    return (value * (max - min)) + min;
}

void mixing(float h_norm, float a_roll_norm, float a_pitch_norm, float a_yaw_norm, float motor_outputs[4]) {
    const float K_ROLL = 0.6;
    const float K_PITCH = 0.6;
    const float K_YAW = 0.0;
    
    motor_outputs[0] = h_norm + K_ROLL * a_roll_norm - K_PITCH * a_pitch_norm + K_YAW * a_yaw_norm;
    motor_outputs[1] = h_norm + K_ROLL * a_roll_norm + K_PITCH * a_pitch_norm - K_YAW * a_yaw_norm;
    motor_outputs[2] = h_norm - K_ROLL * a_roll_norm - K_PITCH * a_pitch_norm - K_YAW * a_yaw_norm;
    motor_outputs[3] = h_norm - K_ROLL * a_roll_norm + K_PITCH * a_pitch_norm + K_YAW * a_yaw_norm; 
}

int power_to_pwm(float power) {
    float pwm = power * ((float)(PWM_RANGE_MAX - PWM_RANGE_MIN) / POWER_MAX) + PWM_RANGE_MIN;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > setting.maxPWM) pwm = setting.maxPWM;
    return (int)pwm;
}

void compute_motor_pwm(float pid_height, float pid_roll, float pid_pitch, float pid_yaw) {
    char msg[128];
    // Definizione dei range dei PID
    const float PID_HEIGHT_MIN = 8.5;
    const float PID_HEIGHT_MAX = 12;
    const float PID_ANGLE_MIN = -10;
    const float PID_ANGLE_MAX = 10;
    
    
    float h_norm;
    if(setting.isAutomatic){
        h_norm      = normalize(pid_height, PID_HEIGHT_MIN, PID_HEIGHT_MAX);
    }else{
        h_norm = pid_height;
    }
    float a_roll_norm = normalize(pid_roll,   -1000,  1000);
    float a_pitch_norm= normalize(pid_pitch,  -1000,  1000);
    float a_yaw_norm  = normalize(pid_yaw,    -1000,  1000);

    char h_norm_str[10], a_roll_norm_str[10], a_pitch_norm_str[10], a_yaw_norm_str[10];
    dtostrf(h_norm, 6, 2, h_norm_str);
    dtostrf(a_roll_norm, 6, 2, a_roll_norm_str);
    dtostrf(a_pitch_norm,6, 2, a_pitch_norm_str);
    dtostrf(a_yaw_norm,  6, 2, a_yaw_norm_str);

    snprintf(msg, sizeof(msg),
             "after pid norm: throttle=%s, a_roll_norm=%s, a_pitch_norm=%s, a_yaw_norm=%s",
             h_norm_str, a_roll_norm_str, a_pitch_norm_str, a_yaw_norm_str);
    sendDebug(msg);
    
    
    #define MIN_THRUST 0
    #define MAX_THRUST 2000
    
    float motor_outputs[4];
    mixing(h_norm, a_roll_norm, a_pitch_norm, a_yaw_norm, motor_outputs);

    char m1_str_mix[10], m2_str_mix[10], m3_str_mix[10], m4_str_mix[10];
    dtostrf(motor_outputs[0], 6, 2, m1_str_mix);
    dtostrf(motor_outputs[1], 6, 2, m2_str_mix);
    dtostrf(motor_outputs[2], 6, 2, m3_str_mix);
    dtostrf(motor_outputs[3], 6, 2, m4_str_mix);

    snprintf(msg, sizeof(msg),
             "after mixing: M1=%s, M2=%s, M3=%s, M4=%s",
             m1_str_mix, m2_str_mix, m3_str_mix, m4_str_mix);
    sendDebug(msg);

    // SUPPONIAMO TRA -1 e 1 MA NON LO é!
    /*motor_outputs[0] = normalize(motor_outputs[0],   -1,  1);
    motor_outputs[1] = normalize(motor_outputs[1],   -1,  1);
    motor_outputs[2] = normalize(motor_outputs[2],   -1,  1);
    motor_outputs[3] = normalize(motor_outputs[3],   -1,  1);
    

    char m1_str[10], m2_str[10], m3_str[10], m4_str[10];
    dtostrf(motor_outputs[0], 6, 2, m1_str);
    dtostrf(motor_outputs[1], 6, 2, m2_str);
    dtostrf(motor_outputs[2], 6, 2, m3_str);
    dtostrf(motor_outputs[3], 6, 2, m4_str);

    snprintf(msg, sizeof(msg),
             "after mixing and norm: M1=%s, M2=%s, M3=%s, M4=%s",
             m1_str, m2_str, m3_str, m4_str);
    sendDebug(msg);
    */
    /*
    float motor_power[4];
    for (int i = 0; i < 4; i++) {
        motor_power[i] = denormalize(motor_outputs[i], POWER_MIN, POWER_MAX);
        if (motor_power[i] < POWER_MIN) motor_power[i] = POWER_MIN;
        if (motor_power[i] > POWER_MAX) motor_power[i] = POWER_MAX;
    }
    */

    /*char mp1_str[10], mp2_str[10], mp3_str[10], mp4_str[10];
    dtostrf(motor_power[0], 6, 2, mp1_str);
    dtostrf(motor_power[1], 6, 2, mp2_str);
    dtostrf(motor_power[2], 6, 2, mp3_str);
    dtostrf(motor_power[3], 6, 2, mp4_str);

    snprintf(msg, sizeof(msg),
             "motor power: M1=%s, M2=%s, M3=%s, M4=%s",
             mp1_str, mp2_str, mp3_str, mp4_str);
    sendDebug(msg);*/
/*
    for (int i = 0; i < 4; i++) {
        pw.motor_pwm[i] = (int)(denormalize(motor_outputs[i]), 1120.0, (float)setting.maxPWM));
    }*/
}


