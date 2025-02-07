#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "sensors.h"
#include "flight_fsm.h"

FlightState currentState = STATE_WAIT;

// Variabile per l'interpolazione del setpoint

interpol_values v;

float t_since_event;

extern void drone_control();  
extern void sendStatus(FlightState state); 

void changeState(FlightState state){
    currentState = state;
    sendStatus(state);
}

void flight_fsm_transition(char* s) {
    switch (currentState) {
    
        case STATE_WAIT:
            if (strcmp(s, "READY") == 0) {
                changeState(STATE_READY);
            }
            break;
            
        case STATE_READY:
            if (strcmp(s, "START") == 0) {
                t_since_event = 0;
                changeState(STATE_FLIGHT);
            }
            break;
            
        case STATE_FLIGHT:
            if (strcmp(s, "STOP") == 0) {
                changeState(STATE_WAIT);
            }
            if(currentTelemetryData.altitude > v.new_setpoint - 0.05 &&
               currentTelemetryData.altitude < v.new_setpoint + 0.05){
            	  changeState(STATE_HOVER);
            } 
            break;
            
        case STATE_HOVER:
            if (strcmp(s, "STOP") == 0) {
                changeState(STATE_WAIT);
            }

            if(currentTelemetryData.altitude < v.new_setpoint - 0.05 ||
               currentTelemetryData.altitude > v.new_setpoint + 0.05){
            	  t_since_event = 0;
                changeState(STATE_FLIGHT);
            } 
            break;
    }
}

extern void init_esc();
extern void init_sensors();
extern void init_pid_controllers();
void handleWait() {
    init_esc();
    init_sensors();
    init_pid_controllers();

    v.old_setpoint = 0.0f;
    v.new_setpoint = 0.0f;

    flight_fsm_transition((char*)"READY");
}

extern void processIncomingSerial();
void handleReady(){
    processIncomingSerial();
}

//extern float interpolate((v.old_setpoint, v.new_setpoint, t_since_event);
extern void get_telemetry();
extern void sendTelemetry();
void handleFlight(){
    processIncomingSerial();

    get_telemetry();
    
    float alt_curr_setpoint = 0;
    alt_curr_setpoint = interpolate(v.old_setpoint, v.new_setpoint, t_since_event);
    altitude_pid.setpoint = alt_curr_setpoint;
    t_since_event += 0.01;
    drone_control();

    sendTelemetry();
    //sendMotor();
}

void handleHover(){
    processIncomingSerial();
    get_telemetry();
    drone_control();
    sendTelemetry();
    //sendMotor();
}
