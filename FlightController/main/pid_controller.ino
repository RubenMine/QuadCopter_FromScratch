#include "pid.h"

void pid_init(PID* pid, int kp, int ki, int kd, float UpperLimit, float LowerLimit, float setpoint, bool a_windup);
void pid_init(PID* pid, int kp, int ki, int kd, float UpperLimit, float LowerLimit, float setpoint, bool a_windup) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Filter_STATE = 0.0f;
    pid->Integrator_STATE = 0.0f;
    pid->UpperLimit = UpperLimit;
    pid->LowerLimit = LowerLimit;
    pid->setpoint = setpoint;
    pid->anti_windup = a_windup;
    
    //char msg[100];
    //snprintf_P(msg, sizeof(msg), PSTR("pid_init: Kp=%d, Ki=%d, Kd=%d, UpperLimit=%f, LowerLimit=%f, setpoint=%f, anti_windup=%d"),
    //           kp, ki, kd, UpperLimit, LowerLimit, setpoint, a_windup);
    //sendDebug(msg);
}

void pid_compute(PID* pid, float sensor_value);
void pid_compute(PID* pid, float sensor_value) {
    float error = pid->setpoint - sensor_value;
    float FilterCoefficient = (pid->Kd * error - pid->Filter_STATE) * 100.0;
    float Sum = (pid->Kp * error + pid->Integrator_STATE) + FilterCoefficient;
    
    
    if (Sum > pid->UpperLimit)
        pid->output = pid->UpperLimit;
    else if (Sum < pid->LowerLimit)
        pid->output = pid->LowerLimit;
    else
        pid->output = Sum;
        
    float anti_windup = pid->anti_windup ? (pid->output - Sum) * 2.0 : 0;
    pid->Integrator_STATE += (anti_windup + pid->Ki * error) * 0.01;
    pid->Filter_STATE += 0.01 * FilterCoefficient;

    char sens[10], err[10], out[10], set[10];
    dtostrf(sensor_value, 6, 2, sens);
    dtostrf(error, 6, 2, err);
    dtostrf(pid->output,6, 2, out);
    dtostrf(pid->setpoint,  6, 2, set);

    char msg[150];
    snprintf(msg, sizeof(msg), "pid_compute: sensor_value=%s, error=%s, output=%s, setpoint=%s",
                              sens, err, out, set);
    sendDebug(msg);
}


extern PID roll_pid, pitch_pid, yaw_pid, altitude_pid;
extern TelemetryData currentTelemetryData;

void drone_control() {

    // --------- COMPUTE PIDS --------- // 
    pid_compute(&roll_pid, currentTelemetryData.roll);
    pid_compute(&pitch_pid, currentTelemetryData.pitch);
    pid_compute(&yaw_pid, currentTelemetryData.yaw);
    // Output Pids x:   -500 < x < 500


    // --------- MIXING MOTOR --------- // 
    float motor_outputs[4];
    const float K_ROLL = 1;
    const float K_PITCH = 1;
    const float K_YAW = 0;
    
    float base_thrust = setting.throttle * (0.9 * setting.maxPWM);
    //if (base_thrust < setting.minPWM){ base_thrust = setting.minPWM; }

    pw.motor_pwm[0] = setting.minPWM + K_ROLL * roll_pid.output - K_PITCH * pitch_pid.output + K_YAW * yaw_pid.output;
    pw.motor_pwm[1] = setting.minPWM + K_ROLL * roll_pid.output + K_PITCH * pitch_pid.output - K_YAW * yaw_pid.output;
    pw.motor_pwm[2] = setting.minPWM - K_ROLL * roll_pid.output - K_PITCH * pitch_pid.output - K_YAW * yaw_pid.output;
    pw.motor_pwm[3] = setting.minPWM - K_ROLL * roll_pid.output + K_PITCH * pitch_pid.output + K_YAW * yaw_pid.output; 

    for(int i = 0; i<4; i++){
      if(pw.motor_pwm[i] > setting.maxPWM){ pw.motor_pwm[i] = setting.maxPWM; }
      if(pw.motor_pwm[i] < setting.minPWM){ pw.motor_pwm[i] = setting.minPWM; }
    }

    motor_power();

    /*
    if(setting.isAutomatic){
      pid_compute(&altitude_pid, currentTelemetryData.altitude);
      compute_motor_pwm(altitude_pid.output, roll_pid.output, pitch_pid.output, yaw_pid.output);
    }else{
      compute_motor_pwm(setting.throttle, roll_pid.output, pitch_pid.output, yaw_pid.output);
    }
    */
}





