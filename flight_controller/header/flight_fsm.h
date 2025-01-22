typedef enum {
    STATE_WAIT,
    STATE_READY,
    STATE_FLIGHT
} FlightState;

static FlightState currentState = STATE_WAIT;