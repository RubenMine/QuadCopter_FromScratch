#include <stdio.h>
#include <string.h>
#include "pid_controller.h"
#include "sensors.h"

typedef enum {
    STATE_WAIT,
    STATE_IDLE,
    STATE_TAKEOFF,
    STATE_FLIGHT
} FlightState;

void flight_fsm_run(FlightState *current_state) {
    switch (*current_state) {
        case STATE_WAIT:
            printf("Waiting for command...\n");
            // Logica per passare a IDLE
            *current_state = STATE_IDLE;
            break;
        case STATE_IDLE:
            printf("Idle: Sending telemetry.\n");
            // Passa a Takeoff su comando
            *current_state = STATE_TAKEOFF;
            break;
        case STATE_TAKEOFF:
            printf("Taking off...\n");
            *current_state = STATE_FLIGHT;
            break;
        case STATE_FLIGHT:
            printf("In Flight.\n");
            break;
    }
}
