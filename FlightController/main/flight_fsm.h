// FSM.h
#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

// Possibili stati di volo
typedef enum FlightState {
    STATE_WAIT,
    STATE_READY,
    STATE_FLIGHT, 
    STATE_HOVER
} FlightState;

extern FlightState currentState;

typedef struct {
    float old_setpoint;
    float new_setpoint;
} interpol_values;
extern interpol_values v;


// Prototipi delle funzioni FSM e di gestione comandi
void changeState(FlightState state);
void flight_fsm_transition(char* cmd);
void handle_command();
//void set_value_of(char* info, float value);
//void flight_fsm_run();
void handleWait();
void handleReady();
void handleFlight();
void handleHover();

#endif

