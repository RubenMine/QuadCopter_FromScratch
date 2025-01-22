#include <stdio.h>
#include <string.h>
#include "pid_controller.h"
#include "sensors.h"


typedef enum {
    STATE_WAIT,
    STATE_READY,
    STATE_FLIGHT
} FlightState;

FlightState currentState = STATE_WAIT;

void flight_fsm_run() {
    switch (currentState) {
    
        case STATE_WAIT:
            Serial.println("[STATE] WAIT");
            handleWait();
            break;
            
        case STATE_READY:
            Serial.println("[STATE] READY");
            handleReady()
            break;
            
        case STATE_FLIGHT:
            Serial.println("[STATE] FLIGHT");
            handleFlight()
            break;
    }
}


void flight_fsm_transition(char* command) {
    switch (currentState) {

        case STATE_WAIT:
            if (strcmp(command, "READY") == 0) {
                currentState = STATE_READY;
            }
            break;
            
        case STATE_READY:
            if (strcmp(command, "START") == 0) {
                currentState = STATE_FLIGHT;
            }
            break;
            
        case STATE_FLIGHT:
            if (strcmp(command, "STOP") == 0) {
                currentState = STATE_WAIT;
            }
            break;
    }
}

void handleWait() {
    Serial.println("[DEBUG] INIT PROCEDURE STARTED");
    
    init_communication();
    init_sensors();
    init_esc();
    init_pid_controllers() 
    
    Serial.println("[DEBUG] INIT PROCEDURE COMPLETED!");    
    
    
    // Change State
    flight_fsm_transition("READY"); 
}

void handleReady(){
    if(read_command()){ handle_command(); }
}

void handleFlight(){
    if(read_command()){ handle_command(); } 
    get_telemetry();
    //drone_control();
    drone_motor();
    send_telemetry();
}
